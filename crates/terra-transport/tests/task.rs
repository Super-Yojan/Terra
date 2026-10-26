use terra_transport::is_control_topic;

#[test]
fn task_topics_are_operator_intent_and_status_is_not() {
    assert!(is_control_topic("task"));
    assert!(is_control_topic("task/action"));
    assert!(!is_control_topic("task/status"));
}
