use std::env;

fn main() {
    let key = "OSR_SOURCE_DIR";
    unsafe {
        env::set_var(key, "../../osr");
    }
    assert_eq!(env::var(key), Ok("../../osr".to_string()));
    // Exported by osr-ffi's links="catenary_osr_bridge" build metadata.
    #[cfg(target_os = "linux")]
    {
        let dir = env::var("DEP_CATENARY_OSR_BRIDGE_LIB_DIR")
            .expect("osr-ffi must be a direct dependency of this crate");

        // Relocatable production layout: <binary dir>/lib/libcatenary_osr_bridge.so.
        println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/lib");
        // Local development: work with Cargo's prebuilt native library.
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
