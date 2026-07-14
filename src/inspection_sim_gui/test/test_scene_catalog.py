import json

import pytest

from inspection_sim_gui.main_window import load_scene_catalog


def test_load_scene_catalog_returns_scenes(tmp_path):
    catalog_path = tmp_path / "scenes.json"
    catalog_path.write_text(
        json.dumps({"scenes": [{"name": "test scene", "world": "test.sdf"}]}),
        encoding="utf-8",
    )

    scenes = load_scene_catalog(catalog_path)

    assert scenes == [{"name": "test scene", "world": "test.sdf"}]


@pytest.mark.parametrize(
    "content, expected_error",
    [
        ("not json", json.JSONDecodeError),
        (json.dumps({"scenes": {"name": "invalid"}}), ValueError),
    ],
)
def test_load_scene_catalog_rejects_invalid_content(
    tmp_path, content, expected_error
):
    catalog_path = tmp_path / "scenes.json"
    catalog_path.write_text(content, encoding="utf-8")

    with pytest.raises(expected_error):
        load_scene_catalog(catalog_path)
