fn main() {
    println!("cargo:rerun-if-env-changed=TERRA_GIT_REVISION");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=../../.git");
    let revision = std::env::var("TERRA_GIT_REVISION")
        .ok()
        .or_else(|| {
            std::process::Command::new("git")
                .args(["rev-parse", "HEAD"])
                .output()
                .ok()
                .filter(|o| o.status.success())
                .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        })
        .unwrap_or_else(|| "unknown".into());
    println!("cargo:rustc-env=TERRA_RUNTIME_REVISION={revision}");
}
