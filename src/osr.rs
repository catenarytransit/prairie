//Written by Kyler Chin

use std::{
    env,
    path::PathBuf,
    sync::{Arc, mpsc},
    time::Instant,
};

use anyhow::{Context, Result, anyhow};
use axum::{
    Json, Router,
    extract::State,
    http::StatusCode,
    response::IntoResponse,
    routing::{get, post},
};
use osr_ffi::{Algorithm, Profile, ffi};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::sync::oneshot;
use tower_http::services::ServeDir;

const PROFILES: &[&str] = &[
    "foot",
    "wheelchair",
    "bike",
    "bike_fast",
    "bike_elevation_low",
    "bike_elevation_high",
    "car",
    "hgv",
    "car_drop_off",
    "car_drop_off_wheelchair",
    "car_parking",
    "car_parking_wheelchair",
    "bus",
    "railway",
    "ferry",
];

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct Point {
    lat: f64,
    lon: f64,
}

impl Point {
    fn validate(self) -> Result<(), &'static str> {
        if !self.lat.is_finite() || !self.lon.is_finite() {
            return Err("latitude and longitude must be finite");
        }
        if !(-90.0..=90.0).contains(&self.lat) || !(-180.0..=180.0).contains(&self.lon) {
            return Err("latitude or longitude out of range");
        }
        Ok(())
    }

    fn native(self) -> ffi::Coordinate {
        ffi::Coordinate::outdoor(self.lat, self.lon)
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum RoutingProfile {
    #[default]
    Foot,
    Wheelchair,
    Bike,
    BikeFast,
    BikeElevationLow,
    BikeElevationHigh,
    Car,
    Hgv,
    CarDropOff,
    CarDropOffWheelchair,
    CarParking,
    CarParkingWheelchair,
    Bus,
    Railway,
    Ferry,
}

impl RoutingProfile {
    fn native(self) -> Profile {
        match self {
            Self::Foot => Profile::Foot,
            Self::Wheelchair => Profile::Wheelchair,
            Self::Bike => Profile::Bike,
            Self::BikeFast => Profile::BikeFast,
            Self::BikeElevationLow => Profile::BikeElevationLow,
            Self::BikeElevationHigh => Profile::BikeElevationHigh,
            Self::Car => Profile::Car,
            Self::Hgv => Profile::Hgv,
            Self::CarDropOff => Profile::CarDropOff,
            Self::CarDropOffWheelchair => Profile::CarDropOffWheelchair,
            Self::CarParking => Profile::CarParking,
            Self::CarParkingWheelchair => Profile::CarParkingWheelchair,
            Self::Bus => Profile::Bus,
            Self::Railway => Profile::Railway,
            Self::Ferry => Profile::Ferry,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum RoutingAlgorithm {
    Dijkstra,
    #[default]
    BidirectionalAStar,
}

impl RoutingAlgorithm {
    fn native(self) -> Algorithm {
        match self {
            Self::Dijkstra => Algorithm::Dijkstra,
            Self::BidirectionalAStar => Algorithm::BidirectionalAStar,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
struct RouteRequest {
    from: Point,
    to: Point,
    #[serde(default)]
    profile: RoutingProfile,
    #[serde(default)]
    algorithm: RoutingAlgorithm,
    max_cost: Option<u32>,
    max_match_distance_meters: Option<f64>,
    #[serde(default)]
    debug_snapping: bool,
    #[serde(default)]
    blocked_osm_nodes: Vec<i64>,
}

impl RouteRequest {
    fn validate(&self) -> Result<(), &'static str> {
        self.from.validate()?;
        self.to.validate()?;
        if self.max_cost.is_some_and(|v| !(1..=1_000_000).contains(&v)) {
            return Err("max_cost must be between 1 and 1000000");
        }
        if self
            .max_match_distance_meters
            .is_some_and(|v| !v.is_finite() || !(1.0..=2_000.0).contains(&v))
        {
            return Err("max_match_distance_meters must be between 1 and 2000");
        }
        if self.blocked_osm_nodes.len() > 128 {
            return Err("no more than 128 blocked OSM node IDs may be supplied");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize)]
struct GraphInfo {
    node_count: u64,
    way_count: u64,
    has_elevations: bool,
}

#[derive(Serialize)]
struct GeoJsonGeometry {
    #[serde(rename = "type")]
    kind: &'static str,
    coordinates: Vec<Vec<[f64; 2]>>,
}

#[derive(Serialize)]
struct RouteSegment {
    osm_way_id: Option<String>,
    from_node_index: u32,
    to_node_index: u32,
    mode_code: u8,
    distance_meters: u32,
    duration_seconds: u32,
    cost: u32,
    elevation_up: u32,
    elevation_down: u32,
    geometry_points: usize,
}

#[derive(Serialize)]
struct SnapDiagnostic {
    osm_way_id: String,
    distance_to_way_meters: f64,
}

#[derive(Serialize)]
struct Diagnostics {
    queue_ms: f64,
    routing_ms: f64,
    snapping_ms: f64,
    pre_response_ms: f64,
    segment_count: usize,
    geometry_point_count: usize,
    graph: GraphInfo,
    from_candidates: Vec<SnapDiagnostic>,
    to_candidates: Vec<SnapDiagnostic>,
    snapping_error: Option<String>,
}

#[derive(Serialize)]
struct RouteReply {
    found: bool,
    duration_seconds: Option<u32>,
    distance_meters: Option<f64>,
    cost: Option<u32>,
    uses_elevator: Option<bool>,
    elevation_up: Option<u32>,
    elevation_down: Option<u32>,
    geometry: Option<GeoJsonGeometry>,
    segments: Vec<RouteSegment>,
    diagnostics: Diagnostics,
}

impl RouteReply {
    fn from_native(
        result: ffi::RouteResult,
        graph: GraphInfo,
        queue_ms: f64,
        routing_ms: f64,
    ) -> Self {
        let mut lines = Vec::new();
        let mut segments = Vec::with_capacity(result.segments.len());
        let mut points = 0;
        for segment in &result.segments {
            let coords: Vec<[f64; 2]> = segment
                .geometry
                .iter()
                .map(|point| [point.lon, point.lat])
                .collect();
            points += coords.len();
            if coords.len() >= 2 {
                lines.push(coords);
            }
            segments.push(RouteSegment {
                osm_way_id: segment
                    .has_osm_way_id
                    .then(|| segment.osm_way_id.to_string()),
                from_node_index: segment.from_node,
                to_node_index: segment.to_node,
                mode_code: segment.mode,
                distance_meters: segment.distance_meters,
                duration_seconds: segment.duration_seconds,
                cost: segment.cost,
                elevation_up: segment.elevation_up,
                elevation_down: segment.elevation_down,
                geometry_points: segment.geometry.len(),
            });
        }

        let found = result.found;
        let segment_count = segments.len();
        Self {
            found,
            duration_seconds: found.then_some(result.duration_seconds),
            distance_meters: found.then_some(result.distance_meters),
            cost: found.then_some(result.cost),
            uses_elevator: found.then_some(result.uses_elevator),
            elevation_up: found.then_some(result.elevation_up),
            elevation_down: found.then_some(result.elevation_down),
            geometry: (!lines.is_empty()).then_some(GeoJsonGeometry {
                kind: "MultiLineString",
                coordinates: lines,
            }),
            segments,
            diagnostics: Diagnostics {
                queue_ms,
                routing_ms,
                snapping_ms: 0.0,
                pre_response_ms: 0.0,
                segment_count,
                geometry_point_count: points,
                graph,
                from_candidates: Vec::new(),
                to_candidates: Vec::new(),
                snapping_error: None,
            },
        }
    }
}

fn nearest_candidates(
    router: &ffi::Router,
    point: Point,
    radius: f64,
) -> Result<Vec<SnapDiagnostic>> {
    let mut candidates: Vec<_> = router
        .snap(&point.native(), radius, false)?
        .into_iter()
        .collect();
    candidates.sort_by(|a, b| {
        a.distance_to_way_meters
            .total_cmp(&b.distance_to_way_meters)
    });
    Ok(candidates
        .into_iter()
        .take(5)
        .map(|c| SnapDiagnostic {
            osm_way_id: c.osm_way_id.to_string(),
            distance_to_way_meters: c.distance_to_way_meters,
        })
        .collect())
}

struct Work {
    enqueued_at: Instant,
    request: RouteRequest,
    reply: oneshot::Sender<Result<RouteReply, String>>,
}

#[derive(Clone)]
struct AppState {
    sender: mpsc::SyncSender<Work>,
    graph: GraphInfo,
}

fn start_worker(data_dir: PathBuf) -> Result<AppState> {
    let (sender, receiver) = mpsc::sync_channel::<Work>(64);
    let (ready_sender, ready_receiver) = mpsc::sync_channel::<Result<GraphInfo, String>>(1);
    std::thread::Builder::new()
        .name("osr-native-routing".into())
        .spawn(move || {
            // Keep the non-Sync C++ Router entirely inside this one worker.
            let path = data_dir.to_string_lossy();
            let router = match ffi::open_router(&path) {
                Ok(router) => router,
                Err(error) => {
                    let _ = ready_sender.send(Err(error.to_string()));
                    return;
                }
            };
            let Some(router) = router.as_ref() else {
                let _ = ready_sender.send(Err("native OSR returned a null router".into()));
                return;
            };
            let graph = GraphInfo {
                node_count: router.node_count(),
                way_count: router.way_count(),
                has_elevations: router.has_elevations(),
            };
            if ready_sender.send(Ok(graph.clone())).is_err() {
                return;
            }
            for work in receiver {
                let queue_ms = work.enqueued_at.elapsed().as_secs_f64() * 1000.0;
                let mut options = ffi::RoutingOptions::default()
                    .profile(work.request.profile.native())
                    .algorithm(work.request.algorithm.native());
                options.max_cost = work.request.max_cost.unwrap_or(86_400);
                options.max_match_distance_meters =
                    work.request.max_match_distance_meters.unwrap_or(100.0);
                let start = Instant::now();
                let route = router.one_to_one(
                    &work.request.from.native(),
                    &work.request.to.native(),
                    &options,
                    &work.request.blocked_osm_nodes,
                );
                let routing_ms = start.elapsed().as_secs_f64() * 1000.0;
                let response = route.map_err(|e| e.to_string()).map(|result| {
                    let mut result =
                        RouteReply::from_native(result, graph.clone(), queue_ms, routing_ms);
                    if work.request.debug_snapping {
                        let snap_start = Instant::now();
                        let radius = options.max_match_distance_meters;
                        let from = nearest_candidates(router, work.request.from, radius);
                        let to = nearest_candidates(router, work.request.to, radius);
                        result.diagnostics.snapping_ms =
                            snap_start.elapsed().as_secs_f64() * 1000.0;
                        match (from, to) {
                            (Ok(a), Ok(b)) => {
                                result.diagnostics.from_candidates = a;
                                result.diagnostics.to_candidates = b;
                            }
                            (a, b) => {
                                result.diagnostics.snapping_error = Some(format!(
                                    "origin: {:?}; destination: {:?}",
                                    a.err(),
                                    b.err()
                                ));
                            }
                        }
                    }
                    result.diagnostics.pre_response_ms =
                        work.enqueued_at.elapsed().as_secs_f64() * 1000.0;
                    tracing::info!(
                        found = result.found,
                        profile = ?work.request.profile,
                        algorithm = ?work.request.algorithm,
                        routing_ms,
                        queue_ms,
                        distance_meters = ?result.distance_meters,
                        "OSR route completed"
                    );
                    result
                });
                let _ = work.reply.send(response);
            }
        })
        .context("spawn OSR worker")?;
    let graph = ready_receiver
        .recv()
        .context("OSR worker terminated during initialization")?
        .map_err(|e| anyhow!("opening OSR data failed: {e}"))?;
    Ok(AppState { sender, graph })
}

type ApiError = (StatusCode, Json<Value>);

fn error(status: StatusCode, message: impl AsRef<str>) -> ApiError {
    (status, Json(json!({ "error": message.as_ref() })))
}

async fn route(
    State(state): State<Arc<AppState>>,
    Json(request): Json<RouteRequest>,
) -> Result<Json<RouteReply>, ApiError> {
    request
        .validate()
        .map_err(|e| error(StatusCode::BAD_REQUEST, e))?;

    let (reply, receiver) = oneshot::channel();
    state
        .sender
        .try_send(Work {
            enqueued_at: Instant::now(),
            request,
            reply,
        })
        .map_err(|e| match e {
            mpsc::TrySendError::Full(_) => {
                error(StatusCode::TOO_MANY_REQUESTS, "routing queue full")
            }
            mpsc::TrySendError::Disconnected(_) => error(
                StatusCode::SERVICE_UNAVAILABLE,
                "routing worker unavailable",
            ),
        })?;

    let result = receiver
        .await
        .map_err(|_| error(StatusCode::SERVICE_UNAVAILABLE, "routing worker stopped"))?;
    result
        .map(Json)
        .map_err(|e| error(StatusCode::UNPROCESSABLE_ENTITY, e))
}

async fn info(State(state): State<Arc<AppState>>) -> Json<Value> {
    Json(json!({
        "status": "ready",
        "graph": state.graph,
        "profiles": PROFILES,
        "algorithms": ["dijkstra", "bidirectional_a_star"],
        "source": "MOTIS OSR via osr-ffi"
    }))
}

async fn health() -> impl IntoResponse {
    Json(json!({ "status": "ok" }))
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "osr_tester=info".into()),
        )
        .init();

    let data_dir = PathBuf::from(
        env::var("OSR_DATA_DIR")
            .context("OSR_DATA_DIR must point to an osr-extract output folder")?,
    );
    if !data_dir.is_dir() {
        return Err(anyhow!(
            "OSR_DATA_DIR is not a directory: {}",
            data_dir.display()
        ));
    }
    let state = Arc::new(start_worker(data_dir)?);
    tracing::info!(?state.graph, "OSR routing graph loaded");

    let static_dir = env::var("STATIC_DIR").unwrap_or_else(|_| "frontend/dist".into());
    let app = Router::new()
        .route("/api/health", get(health))
        .route("/api/info", get(info))
        .route("/api/route", post(route))
        .fallback_service(ServeDir::new(static_dir))
        .with_state(state);

    let bind = env::var("BIND_ADDR").unwrap_or_else(|_| "127.0.0.1:3000".into());
    let listener = tokio::net::TcpListener::bind(&bind).await?;
    tracing::info!(%bind, "OSR tester listening");
    axum::serve(listener, app).await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coordinate_validation_rejects_out_of_range() {
        assert!(
            Point {
                lat: 33.6,
                lon: -117.8
            }
            .validate()
            .is_ok()
        );
        assert!(
            Point {
                lat: 91.0,
                lon: 0.0
            }
            .validate()
            .is_err()
        );
        assert!(
            Point {
                lat: 0.0,
                lon: -181.0
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn request_bounds_are_enforced() {
        let mut req: RouteRequest = serde_json::from_value(json!({
            "from": {"lat": 33.6, "lon": -117.8},
            "to": {"lat": 33.7, "lon": -117.9}
        }))
        .unwrap();
        assert!(req.validate().is_ok());
        req.max_cost = Some(0);
        assert!(req.validate().is_err());
        req.max_cost = Some(1000);
        req.max_match_distance_meters = Some(f64::NAN);
        assert!(req.validate().is_err());
    }

    #[test]
    fn profile_and_algorithm_use_public_codes() {
        let req: RouteRequest = serde_json::from_value(json!({
            "from": {"lat": 33.6, "lon": -117.8},
            "to": {"lat": 33.7, "lon": -117.9},
            "profile": "wheelchair",
            "algorithm": "dijkstra"
        }))
        .unwrap();
        assert_eq!(req.profile.native() as u8, Profile::Wheelchair as u8);
        assert_eq!(req.algorithm.native() as u8, Algorithm::Dijkstra as u8);
        assert!(
            serde_json::from_value::<RouteRequest>(json!({
                "from": {"lat": 0, "lon": 0},
                "to": {"lat": 1, "lon": 1},
                "profile": "bike_sharing"
            }))
            .is_err()
        );
    }

    #[test]
    fn geometry_is_geojson_order_and_never_invents_straight_line() {
        let mut native = ffi::RouteResult {
            found: true,
            cost: 12,
            duration_seconds: 17,
            distance_meters: 23.0,
            uses_elevator: false,
            elevation_up: 0,
            elevation_down: 0,
            segments: vec![],
        };
        let graph = GraphInfo {
            node_count: 1,
            way_count: 1,
            has_elevations: false,
        };
        let empty = RouteReply::from_native(native.clone(), graph.clone(), 0.0, 1.0);
        assert!(empty.geometry.is_none());
        native.segments.push(ffi::PathSegment {
            osm_way_id: 9,
            has_osm_way_id: true,
            from_node: 0,
            to_node: 1,
            mode: 0,
            from_level: 0.0,
            to_level: 0.0,
            cost: 12,
            duration_seconds: 17,
            distance_meters: 23,
            elevation_up: 0,
            elevation_down: 0,
            geometry: vec![
                ffi::Coordinate::outdoor(33.65, -117.85),
                ffi::Coordinate::outdoor(33.66, -117.84),
            ],
        });
        let result = RouteReply::from_native(native, graph, 0.0, 1.0);
        let line = result.geometry.unwrap().coordinates;
        assert_eq!(line[0][0], [-117.85, 33.65]);
        assert_eq!(line[0][1], [-117.84, 33.66]);
    }
}