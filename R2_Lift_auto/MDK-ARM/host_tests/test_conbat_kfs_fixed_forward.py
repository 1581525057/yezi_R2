from pathlib import Path
import unittest


SOURCE_PATH = Path(__file__).resolve().parents[1] / "TASK" / "conbat_task.cpp"


def case_body(source: str, start: str, end: str) -> str:
    start_marker = f"case {start}:"
    end_marker = f"case {end}:"
    return source.split(start_marker, 1)[1].split(end_marker, 1)[0]


class ConbatKfsFixedForwardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SOURCE_PATH.read_text(encoding="utf-8")

    def assert_fixed_forward_output(self, body: str) -> None:
        self.assertNotIn("dt35.ch2", body)
        self.assertIn("path_active_ = 1U;", body)
        self.assertIn("path_vx_target_ = 0.3f;", body)
        self.assertIn("path_vy_target_ = 0.0f;", body)
        self.assertIn("path_wz_target_ = 0.0f;", body)

    def test_first_wait_uses_fixed_speed_until_event_one(self) -> None:
        body = case_body(
            self.source,
            "PICK_KFS_FIRST_WAIT_DONE",
            "PICK_KFS_FIRST_BACKWARD",
        )

        self.assertIn("arm_comm.rx_data_.event == 1U", body)
        self.assertIn("pick_kfs_step_ = PICK_KFS_FIRST_BACKWARD;", body)
        self.assert_fixed_forward_output(body)

    def test_second_wait_keeps_start_gate_then_uses_fixed_speed(self) -> None:
        body = case_body(
            self.source,
            "PICK_KFS_SECOND_WAIT_READY",
            "PICK_KFS_SECOND_BACKWARD",
        )

        start_gate = body.index("arm_comm.rx_data_.event == 5U")
        fixed_speed = body.index("path_vx_target_ = 0.3f;")
        self.assertLess(start_gate, fixed_speed)
        self.assertIn("arm_comm.rx_data_.event == 1U", body)
        self.assertIn("pick_kfs_step_ = PICK_KFS_SECOND_BACKWARD;", body)
        self.assert_fixed_forward_output(body)

    def test_wait_states_have_no_dt35_target_parameters(self) -> None:
        self.assertNotIn("CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_ACC_MPS2", self.source)
        self.assertNotIn("CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_MAX_MPS", self.source)
        self.assertNotIn("CONBAT_PICK_KFS_FIRST_WAIT_DT35_TARGET_MM", self.source)
        self.assertNotIn("CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM", self.source)


if __name__ == "__main__":
    unittest.main()
