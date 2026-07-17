from pathlib import Path
import unittest


SOURCE_PATH = Path(__file__).resolve().parents[1] / "TASK" / "conbat_task.cpp"


class ConbatKfsLaserSideMappingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        source = SOURCE_PATH.read_text(encoding="utf-8")
        cls.body = source.split(
            "uint8_t CONBAT_TASK::runPickKfsLaserAlign", 1
        )[1].split("uint8_t CONBAT_TASK::runPickKfs(void)", 1)[0]

    def test_blue_uses_right_laser_and_red_uses_left_laser(self) -> None:
        self.assertIn(
            "(field_side_index == 0U) ? laser_right : laser_left",
            self.body,
        )

    def test_error_direction_matches_selected_laser(self) -> None:
        blue_error = self.body.index("? (target_mm - measured_mm)")
        red_error = self.body.index(": (measured_mm - target_mm)")
        self.assertLess(blue_error, red_error)


if __name__ == "__main__":
    unittest.main()
