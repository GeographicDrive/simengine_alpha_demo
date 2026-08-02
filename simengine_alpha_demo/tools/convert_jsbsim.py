#!/usr/bin/env python3
"""
tools/convert_jsbsim.py — JSBSim FDM -> simengine aircraft JSON converter.

This is what makes simengine "FlightGear-format compatible": FlightGear's
JSBSim-based aircraft (like the uploaded A320-family add-on) ship real
mass, geometry, landing-gear, and aerodynamic-derivative data in
JSBSim's <fdm_config> XML schema. This script reads that schema and
writes out a JSON file in simengine's own aircraft-data shape (matching
assets/aircraft/generic_narrowbody.json / AircraftComponent), so a real
JSBSim aircraft definition becomes a real simengine aircraft — not a
copy of FlightGear's code or 3D assets, just a unit-converted read of
its numeric flight-dynamics data.

WHAT THIS CONVERTS:
  - metrics    -> wing area/span/chord (ft, ft^2 -> m, m^2)
  - mass_balance -> mass (lb -> kg), inertia tensor (slug*ft^2 -> kg*m^2)
  - ground_reactions BOGEY contacts -> one GearLegConfig each (location,
    spring/damper coefficients, friction, steering, in JSBSim's
    lb/ft, lb/ft/sec -> N/m, N/(m/s))
  - propulsion/engine[@file] -> reads the referenced turbine_engine's
    <milthrust> (lb -> N) for a static max-thrust estimate
  - aerodynamics axis functions -> for each coefficient simengine's
    AeroCoefficients models as a single linear derivative (e.g. Cmalpha,
    Clda, Cnb...), reads either a JSBSim <value> constant directly, or
    for coefficients JSBSim expresses as a lookup table (e.g. CLalpha
    vs alpha, ganged with flap position), estimates a linear slope from
    the two nearest-to-zero table rows in the clean (0-deg-flap) column.
    JSBSim's tables are frequently non-linear (stall behavior, flap
    effects) — this conversion intentionally throws that non-linearity
    away to fit simengine's current (simpler) linear aero model. See
    docs/ROADMAP.md for the note on eventually giving simengine's
    AeroCoefficients real table support instead of single derivatives.

WHAT THIS DOES NOT CONVERT (out of scope for flight-dynamics
compatibility, and in the 3D-asset case a separate licensing
consideration since those are the add-on's original modeled/textured
artwork, not just numeric facts):
  - 3D meshes (.ac files), textures, sounds, cockpit/instrument/autopilot
    Nasal scripts, or anything under Systems/.

USAGE:
    python3 tools/convert_jsbsim.py \
        /path/to/A320-family-master/A320-211.xml \
        assets/aircraft/a320-211.json \
        --type-name a320-211 \
        --source-note "FlightGear A320-family (GPL), A320-211.xml"
"""

import argparse
import re
import sys
import xml.etree.ElementTree as ET

FT_TO_M = 0.3048
FT2_TO_M2 = FT_TO_M * FT_TO_M
LB_TO_KG = 0.45359237
SLUGFT2_TO_KGM2 = 1.35581795
LBF_TO_N = 4.4482216153
LBF_PER_FT_TO_N_PER_M = LBF_TO_N / FT_TO_M  # spring: lb/ft -> N/m
DEG_TO_RAD = 3.14159265358979 / 180.0


def text_of(elem, path, default=None, cast=float):
    node = elem.find(path)
    if node is None or node.text is None:
        return default
    return cast(node.text.strip())


def find_function(aero_root, name):
    for fn in aero_root.iter('function'):
        if fn.get('name') == name:
            return fn
    return None


def constant_value(fn):
    """Return the trailing <value> in a <product> if present (JSBSim's
    way of expressing 'coefficient * qbar * S * ...')."""
    if fn is None:
        return None
    val = fn.find('.//value')
    if val is not None and val.text is not None:
        try:
            return float(val.text.strip())
        except ValueError:
            return None
    return None


def table_slope_near_zero(fn):
    """For a <table> keyed on alpha-rad or beta-rad (optionally with a
    second column axis like flap position), estimate d(coeff)/d(axis)
    using the clean/first-column rows nearest to axis=0. Returns None if
    no table is present."""
    if fn is None:
        return None
    table = fn.find('.//table')
    if table is None:
        return None
    data = table.find('tableData')
    if data is None or data.text is None:
        return None
    lines = [ln.strip() for ln in data.text.strip().splitlines() if ln.strip()]
    has_two_axes = table.find('independentVar[@lookup="column"]') is not None
    rows = []
    header_cols = None
    for ln in lines:
        parts = re.split(r'\s+', ln)
        nums = [float(p) for p in parts]
        if has_two_axes and header_cols is None:
            header_cols = nums  # first line is the column header (e.g. flap positions)
            continue
        rows.append(nums)
    if not rows:
        return None
    # First numeric column of each row is the row axis value (alpha/beta);
    # second column (index 1) is the clean/first-flap-setting value.
    rows.sort(key=lambda r: abs(r[0]))
    if len(rows) < 2:
        return None
    (x0, y0), (x1, y1) = (rows[0][0], rows[0][1]), (rows[1][0], rows[1][1])
    if abs(x1 - x0) < 1e-9:
        return None
    return (y1 - y0) / (x1 - x0)


def coefficient(aero_root, name):
    fn = find_function(aero_root, name)
    c = constant_value(fn)
    if c is not None:
        return c
    s = table_slope_near_zero(fn)
    if s is not None:
        return s
    return 0.0


def convert(xml_path, type_name, source_note):
    tree = ET.parse(xml_path)
    root = tree.getroot()

    metrics = root.find('metrics')
    S_ft2 = text_of(metrics, 'wingarea', 0.0)
    b_ft = text_of(metrics, 'wingspan', 0.0)
    c_ft = text_of(metrics, 'chord', 0.0)

    mb = root.find('mass_balance')
    empty_lb = text_of(mb, 'emptywt', 0.0)
    # Add point-masses (crew/pax/cargo placeholders JSBSim ships as a
    # representative loaded weight) to get a plausible operating mass
    # rather than bare empty weight.
    total_lb = empty_lb
    for pm in mb.findall('pointmass'):
        total_lb += text_of(pm, 'weight', 0.0)
    Ixx = text_of(mb, 'ixx', 0.0) * SLUGFT2_TO_KGM2
    Iyy = text_of(mb, 'iyy', 0.0) * SLUGFT2_TO_KGM2
    Izz = text_of(mb, 'izz', 0.0) * SLUGFT2_TO_KGM2
    Ixz = abs(text_of(mb, 'ixz', 0.0)) * SLUGFT2_TO_KGM2

    gear = []
    gr = root.find('ground_reactions')
    if gr is not None:
        for contact in gr.findall('contact'):
            if contact.get('type') != 'BOGEY':
                continue  # skip STRUCTURE (fuselage skid) contacts, not wheels
            name = contact.get('name', 'gear')
            loc = contact.find('location')
            x_in = text_of(loc, 'x', 0.0)
            y_in = text_of(loc, 'y', 0.0)
            z_in = text_of(loc, 'z', 0.0)
            IN_TO_M = 0.0254
            spring = text_of(contact, 'spring_coeff', 0.0) * LBF_PER_FT_TO_N_PER_M
            damper = text_of(contact, 'damping_coeff', 0.0) * LBF_PER_FT_TO_N_PER_M
            max_steer_deg = text_of(contact, 'max_steer', 0.0)
            static_fric = text_of(contact, 'static_friction', 0.5)
            dynamic_fric = text_of(contact, 'dynamic_friction', 0.6)
            rolling_fric = text_of(contact, 'rolling_friction', 0.02)
            has_brake = (contact.findtext('brake_group', 'NONE').strip() != 'NONE')
            gear.append({
                "name": name,
                # simengine's body axes are +x forward from CG, +z down;
                # JSBSim's structural axes here are +x aft from a
                # reference datum, +z up — this converts JSBSim's CG-
                # relative location convention (x,y,z already relative to
                # CG for these files) into simengine's sign convention.
                "attachBody_m": [-x_in * IN_TO_M * 0.0 + 0.0, y_in * IN_TO_M, -z_in * IN_TO_M],
                "attachBody_m_note": "x left at 0.0 pending a verified fuselage-station -> CG-relative-x reference; see ROADMAP",
                "springConstant_Npm": spring,
                "damperConstant_Npmps": damper,
                "maxSteerAngleDeg": max_steer_deg,
                "steerable": max_steer_deg > 0.0,
                "staticFriction": static_fric,
                "dynamicFriction": dynamic_fric,
                "rollingResistance": rolling_fric,
                "hasBrake": has_brake,
            })

    thrust_n_per_engine = None
    prop = root.find('propulsion')
    engine_count = 0
    if prop is not None:
        engines = prop.findall('engine')
        engine_count = len(engines)
        if engines:
            eng_file = engines[0].get('file')
            eng_path = xml_path.rsplit('/', 1)[0] + f'/Engines/{eng_file}.xml'
            try:
                eng_tree = ET.parse(eng_path)
                mil = eng_tree.getroot().find('milthrust')
                if mil is not None and mil.text:
                    thrust_n_per_engine = float(mil.text.strip()) * LBF_TO_N
            except (FileNotFoundError, ET.ParseError):
                pass

    aero_root = root.find('aerodynamics')
    aero = {
        "CD0": coefficient(aero_root, 'aero/coefficient/CDo'),
        "CDde": coefficient(aero_root, 'aero/coefficient/CDde'),
        "CD_gear": coefficient(aero_root, 'aero/coefficient/CDgear'),
        "CD_speedbrake": coefficient(aero_root, 'aero/coefficient/CDspeedbrake'),
        "CYb": coefficient(aero_root, 'aero/coefficient/CYb'),
        "CL0": coefficient(aero_root, 'aero/coefficient/CLalpha'),  # slope fn also carries the alpha=0 intercept; see note below
        "CLDe": coefficient(aero_root, 'aero/coefficient/CLDe'),
        "Clb": coefficient(aero_root, 'aero/coefficient/Clb'),
        "Clp": coefficient(aero_root, 'aero/coefficient/Clp'),
        "Clda": coefficient(aero_root, 'aero/coefficient/Clda'),
        "Cldr": coefficient(aero_root, 'aero/coefficient/Cldr'),
        "Cm0": coefficient(aero_root, 'aero/coefficient/Cmo'),
        "Cmalpha": coefficient(aero_root, 'aero/coefficient/Cmalpha'),
        "CmDe": coefficient(aero_root, 'aero/coefficient/CmDe'),
        "Cmq": coefficient(aero_root, 'aero/coefficient/Cmq'),
        "Cnr": coefficient(aero_root, 'aero/coefficient/Cnr'),
        "Cnb": coefficient(aero_root, 'aero/coefficient/Cnb'),
        "Cndr": coefficient(aero_root, 'aero/coefficient/Cndr'),
    }

    # CLalpha's table encodes both the alpha=0 intercept (this file's
    # row 2, "0.0000") and the slope; table_slope_near_zero() already
    # gave us the slope for the CL0 key above by mistake (name collision
    # in the loop above) — recompute both explicitly and correctly here.
    cl_fn = find_function(aero_root, 'aero/coefficient/CLalpha')
    cl_slope = table_slope_near_zero(cl_fn)
    cl_intercept = None
    if cl_fn is not None:
        table = cl_fn.find('.//table')
        data = table.find('tableData') if table is not None else None
        if data is not None and data.text:
            lines = [ln.strip() for ln in data.text.strip().splitlines() if ln.strip()]
            for ln in lines[1:]:
                nums = [float(p) for p in re.split(r'\s+', ln)]
                if abs(nums[0]) < 1e-6:
                    cl_intercept = nums[1]
                    break
    aero["CL0"] = cl_intercept if cl_intercept is not None else 0.2
    aero["CLa"] = cl_slope if cl_slope is not None else 5.5

    out = {
        "_source": source_note,
        "_converted_by": "tools/convert_jsbsim.py — see docs/ROADMAP.md for conversion caveats",
        "typeName": type_name,
        "mass": {
            "mass_kg": round(total_lb * LB_TO_KG, 1),
            "S_m2": round(S_ft2 * FT2_TO_M2, 3),
            "b_m": round(b_ft * FT_TO_M, 3),
            "c_m": round(c_ft * FT_TO_M, 3),
            "Ix": round(Ixx, 1), "Iy": round(Iyy, 1), "Iz": round(Izz, 1), "Ixz": round(Ixz, 1),
        },
        "aero": aero,
        "propulsion": {
            "type": "Jet",
            "engineCount": engine_count if engine_count else 2,
            "T_max_per_engine_N": round(thrust_n_per_engine, 1) if thrust_n_per_engine else 120000.0,
        },
        "gear": gear,
    }
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input_xml', help='Path to a JSBSim <fdm_config> XML file (e.g. A320-211.xml)')
    ap.add_argument('output_json', help='Path to write the simengine aircraft JSON to')
    ap.add_argument('--type-name', default='converted-aircraft')
    ap.add_argument('--source-note', default='')
    args = ap.parse_args()

    result = convert(args.input_xml, args.type_name, args.source_note)

    import json
    with open(args.output_json, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"Wrote {args.output_json}")
    print(f"  mass={result['mass']['mass_kg']}kg  S={result['mass']['S_m2']}m^2  "
          f"b={result['mass']['b_m']}m  gear_legs={len(result['gear'])}  "
          f"thrust/engine={result['propulsion']['T_max_per_engine_N']}N")


if __name__ == '__main__':
    main()
