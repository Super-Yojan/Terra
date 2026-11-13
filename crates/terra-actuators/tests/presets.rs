//! The editable presets in `presets/` parse and pass structural validation.
#[test]
fn presets_are_structurally_valid() {
    for (name, json) in [
        ("terra-mini", include_str!("../presets/terra-mini.json")),
        ("esc-template", include_str!("../presets/esc-template.json")),
        (
            "mixed-servo-template",
            include_str!("../presets/mixed-servo-template.json"),
        ),
    ] {
        let layout: terra_actuators::Layout = serde_json::from_str(json).expect(name);
        assert_eq!(terra_actuators::validate_structure(&layout), Ok(()), "{name}");
    }
}
