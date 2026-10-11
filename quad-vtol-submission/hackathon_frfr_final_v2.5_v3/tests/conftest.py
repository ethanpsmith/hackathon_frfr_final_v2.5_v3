import pytest


def pytest_addoption(parser):
    parser.addoption("--sitl",action="store_true",help="explicitly enable owned-SITL flight integration tests")


def pytest_collection_modifyitems(config,items):
    if not config.getoption("--sitl"):
        for item in items:
            if "sitl" in item.keywords:
                item.add_marker(pytest.mark.skip(reason="SITL flight requires explicit --sitl"))
