export const SITE_TITLE = "Terra";

/** Prefix an in-site path with the project Page base, `/Terra`. */
export const href = (path: string) =>
    `${import.meta.env.BASE_URL.replace(/\/$/, "")}${path === "/" ? "/" : path}`;

export interface NavPage {
    /** Route without the base, e.g. `/hardware/FLASH_SD`. Full URLs link out. */
    path: string;
    label: string;
}

export interface NavSection {
    title: string;
    /** The section's own page, if it has one. */
    path?: string;
    blurb: string;
    pages: NavPage[];
}

/** The menu: the big links in the overlay. */
export const menu: NavPage[] = [
    { path: "/getting-started", label: "Get started" },
    { path: "/architecture", label: "Architecture" },
    { path: "/phone", label: "TerraPhone" },
    { path: "/hardware", label: "Hardware" },
    { path: "/autonomy", label: "Autonomy" },
    { path: "/#guides", label: "All pages" },
];

/** Every page in reading order, grouped as the old MkDocs nav was. */
export const sections: NavSection[] = [
    {
        title: "Start",
        blurb: "What Terra is, how the pieces fit, and the first run.",
        pages: [
            { path: "/architecture", label: "Architecture" },
            { path: "/getting-started", label: "Getting started" },
            { path: "/zenoh", label: "Zenoh" },
            { path: "/simulation", label: "Simulation" },
        ],
    },
    {
        title: "Crates",
        path: "/crates",
        blurb: "The Rust workspace, one page per crate.",
        pages: [
            { path: "/crates/terra-types", label: "terra-types" },
            { path: "/crates/terra-state", label: "terra-state" },
            { path: "/crates/terra-control", label: "terra-control" },
            { path: "/crates/terra-waypoint", label: "terra-waypoint" },
            { path: "/crates/terra-transport", label: "terra-transport" },
            { path: "/crates/fpv-signaling", label: "fpv-signaling" },
            { path: "/crates/terra-mapping", label: "terra-mapping" },
            { path: "/crates/terra-navigation", label: "terra-navigation" },
            { path: "/crates/terra-exploration", label: "terra-exploration" },
            { path: "/crates/terra-autonomy", label: "terra-autonomy" },
            { path: "/crates/terra-experiment", label: "terra-experiment" },
            { path: "/crates/terra-motors", label: "terra-motors" },
            { path: "/crates/terra-actuators", label: "terra-actuators" },
            { path: "/crates/terra-mobile", label: "terra-mobile" },
            { path: "https://super-yojan.dev/Terra/api/", label: "Rust API" },
        ],
    },
    {
        title: "TerraPhone",
        path: "/phone",
        blurb: "The iPhone and Android app: build, run, pair, calibrate.",
        pages: [
            { path: "/phone/build", label: "Building in Xcode" },
            { path: "/MOBILE_ANDROID", label: "Android" },
            { path: "/phone/simulator", label: "Simulator and iPhone" },
            { path: "/phone/bluetooth", label: "Bluetooth pairing" },
            { path: "/phone/fpv", label: "FPV video" },
            { path: "/phone/tap-to-configure", label: "Tap to configure" },
            { path: "/phone/rover1-configuration", label: "rover1 configuration" },
            { path: "/MOUNT_CALIBRATION_CHECKLIST", label: "Mount calibration" },
        ],
    },
    {
        title: "Hardware",
        path: "/hardware",
        blurb: "The Raspberry Pi relay, the Fusion HAT and the SD card.",
        pages: [
            { path: "/hardware/FLASH_SD", label: "Flash the SD card" },
            { path: "/hardware/INSTALL", label: "Install on a Pi" },
            { path: "/hardware/BLUETOOTH", label: "Bluetooth peripheral" },
            { path: "/hardware/USB", label: "USB link" },
            { path: "/hardware/FUSION_HAT", label: "Fusion HAT" },
            { path: "/hardware/FUNCTIONS_UML", label: "Function UML" },
            { path: "/hardware/BENCH", label: "Bench procedure" },
            { path: "/hardware/evidence", label: "Evidence status" },
            { path: "/hardware/evidence/BINARY_BUILD", label: "Binary build" },
            { path: "/hardware/evidence/SOURCE_REVIEW", label: "Source review" },
            { path: "/REAL_ROVER_TUNING", label: "Real rover tuning" },
        ],
    },
    {
        title: "Autonomy",
        path: "/autonomy",
        blurb: "Levels L0 to L5, the contract, and the evidence.",
        pages: [
            { path: "/autonomy/PROTOCOL", label: "Contract" },
            { path: "/autonomy/TASK", label: "L5 task" },
            { path: "/autonomy/DOCK", label: "Bucket dock" },
            { path: "/autonomy/TARGET_SEARCH", label: "Target search" },
            { path: "/autonomy/outdoor-gps", label: "Outdoor GPS" },
            { path: "/autonomy/turning-policy", label: "Turning policy" },
            { path: "/autonomy/evidence", label: "Evidence" },
            { path: "/autonomy/evidence/l4", label: "L4 evidence" },
            { path: "/l2-l4-implementation", label: "L2–L4 implementation" },
            { path: "/control-safety-requirements-draft", label: "Control safety requirements" },
        ],
    },
    {
        title: "Development",
        blurb: "Working without a Mac, containers, the phone controller and dashboards.",
        pages: [
            { path: "/DEVELOP-WITHOUT-MAC", label: "Without a Mac" },
            { path: "/DOCKER", label: "Containers" },
            { path: "/MOBILE_CONTROL", label: "Phone controller" },
            { path: "/DASHBOARD_TAILSCALE", label: "Dashboard over Tailscale" },
        ],
    },
    {
        title: "Observability",
        blurb: "Logs, events and the Rerun deployments.",
        pages: [
            { path: "/observability-diagnostics", label: "Diagnostics" },
            { path: "/observability/operations", label: "Operations" },
            { path: "/observability/event-schema", label: "Event schema" },
            { path: "/observability/coverage", label: "Coverage" },
            { path: "/observability/implementation-record", label: "Implementation record" },
            { path: "/observability/2026-10-22-connectivity-release", label: "Connectivity release" },
            { path: "/observability/deployments/2026-10-21-direct-devices", label: "Deployment: direct devices" },
            { path: "/observability/deployments/2026-10-21-gcp", label: "Deployment: GCP" },
            { path: "/observability/deployments/2026-10-21-rerun-web", label: "Deployment: Rerun web" },
        ],
    },
];

/** Design notes are listed from the collection, newest last, under this title. */
export const DESIGN_NOTES = {
    title: "Design notes",
    blurb: "Plans and specs written before a feature was built.",
    prefix: "superpowers/",
};

/** The section a page belongs to, if any. */
export const sectionOf = (path: string) =>
    sections.find((s) => s.path === path || s.pages.some((p) => p.path === path));

export const isExternal = (path: string) => /^https?:/.test(path);
/** A nav path as a URL: external as is, in-site with the base and a trailing slash. */
export const link = (path: string) => {
    if (isExternal(path)) return path;
    if (path.startsWith("/#")) return `${href("/")}${path.slice(1)}`;
    return path === "/" ? href("/") : `${href(path)}/`;
};
