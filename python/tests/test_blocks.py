import pytest

import simpel as sp


def test_categories_and_counts():
    assert sp.categories() == ["control", "electrical", "general", "magnetic"]
    allb = sp.blocks()
    assert len(allb) == len(set(allb)) == 196
    assert sum(len(sp.blocks(c)) for c in sp.categories()) == len(allb)
    assert "MOSFET" in sp.blocks("electrical")
    with pytest.raises(ValueError):
        sp.blocks("nope")


def test_block_info_mosfet():
    b = sp.block_info("MOSFET")
    assert b.category == "electrical" and b.in_palette
    assert b.param("Ron").default == "10m"
    assert len(b.pins) == 3
    assert {p.kind for p in b.pins} == {"power", "control"}
    assert "MOSFET" in str(b)
    with pytest.raises(KeyError):
        b.param("nope")


def test_unknown_type_suggests():
    with pytest.raises(KeyError, match="MOSFET"):
        sp.block_info("MOSFT")


def test_corpus_only_block():
    b = sp.block_info("XFMR")
    assert not b.in_palette
    assert b.category == "electrical"


def test_roles_and_legacy():
    legacy = set(sp.blocks(include_legacy=True)) - set(sp.blocks())
    assert len(legacy) == 13 and "JK_FLIP_FLOP" in legacy
    assert "DISCRETE_INT" in sp.blocks(role="unsupported")
    assert "GND" in sp.blocks(role="structural")
    assert "SCOPE" in sp.blocks(role="observer")
    assert "MOSFET" in sp.blocks(role="model")
    assert not sp.block_info("ZOH").simulated
    with pytest.raises(ValueError):
        sp.blocks(role="nope")


def test_alias_resolves_to_block():
    b = sp.block_info("Resistor")
    assert b.type == "R" and "Resistor" in b.aliases


def test_registry_supplied_pins():
    assert [p.name for p in sp.block_info("CONSTANT").pins] == [p.name for p in sp.block_info("CONST").pins]


def test_param_choices():
    b = sp.block_info("RELATIONAL_OPERATOR")
    assert b.param("operator").choices == ("==", "~=", "!=", "<", "<=", ">", ">=")
    assert sp.block_info("R").param("value").choices == ()
    assert "function" in sp.block_info("TRIG_FCN").choices
