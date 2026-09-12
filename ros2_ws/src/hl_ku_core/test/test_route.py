from hl_ku_core.route import Route
import pytest


def test_load_and_query_route(tmp_path):
    path = tmp_path / "route.csv"
    path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.5,NORMAL,1\n"
        "1,1,0,0.4,HILL,1\n"
        "2,2,0,0.2,PERP_PARK,-1\n",
        encoding="utf-8",
    )
    route = Route.load_csv(path)
    assert route.nearest_index(1.1, 0.0) == 1
    assert route.lookahead_index(0, 1.5) == 2
    assert route.waypoints[2].direction == -1


def test_finish_metadata_waits_for_final_point(tmp_path):
    path = tmp_path / "sparse_finish.csv"
    path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.3,NORMAL,1\n"
        "16,16,0,0.0,FINISH,1\n",
        encoding="utf-8",
    )
    route = Route.load_csv(path)

    assert route.nearest_index(9.0, 0.0) == 1
    before_finish = route.metadata_waypoint(1, 9.0, 0.0, 0.5)
    at_finish = route.metadata_waypoint(1, 15.7, 0.0, 0.5)

    assert before_finish.mission == "NORMAL"
    assert before_finish.target_speed_mps == 0.3
    assert at_finish.mission == "FINISH"
    assert at_finish.target_speed_mps == 0.0


@pytest.mark.parametrize(
    "row",
    (
        "1,1,0,-0.4,NORMAL,1\n",
        "1,1,0,0.4,TYPO,1\n",
        "0,1,0,0.4,NORMAL,1\n",
    ),
)
def test_rejects_unsafe_route_values(tmp_path, row):
    path = tmp_path / "route.csv"
    path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.5,NORMAL,1\n" + row,
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        Route.load_csv(path)
