import pytest

from uav_utils.project_paths import project_path, project_root


def make_workspace(root):
    marker = root / 'src/realflight_modules/px4ctrl/package.xml'
    marker.parent.mkdir(parents=True)
    marker.write_text('<package/>')
    return root


@pytest.mark.parametrize('layout', [
    'install/flight_data_recorder/share/flight_data_recorder',
    'install/share/flight_data_recorder',
    'src/realflight_modules/flight_data_recorder',
])
def test_data_paths_follow_workspace_after_move(tmp_path, monkeypatch, layout):
    root = make_workspace(tmp_path / 'original')
    anchor = root / layout
    anchor.mkdir(parents=True, exist_ok=True)
    moved = tmp_path / 'renamed project'
    root.rename(moved)
    elsewhere = tmp_path / 'another_working_directory'
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    for value in ('datalog/flightlog', 'datalog/quadsimlog',
                  'datalog/omtraj/omtraj_optimized.csv'):
        assert project_path(value, moved / layout) == moved / value


def test_symlink_install(tmp_path):
    root = make_workspace(tmp_path / 'workspace')
    source = root / 'src/realflight_modules/flight_data_recorder'
    source.mkdir()
    share = root / 'install/flight_data_recorder/share/flight_data_recorder'
    share.parent.mkdir(parents=True)
    share.symlink_to(source, target_is_directory=True)
    assert project_root(share) == root


def test_absolute_path_does_not_require_source_workspace(tmp_path):
    assert project_path(tmp_path / 'external', tmp_path / 'standalone_install') == tmp_path / 'external'


def test_missing_workspace_is_reported_instead_of_using_cwd(tmp_path, monkeypatch):
    root = make_workspace(tmp_path / 'unrelated_workspace')
    monkeypatch.chdir(root)
    with pytest.raises(RuntimeError, match='Cannot locate'):
        project_path('datalog/flightlog', tmp_path / 'standalone_install')
