# Builds enclosure_v3: slim L-shaped housing with a removable side lid (-Y side, 5x M3 Torx
# screws into heat-set inserts), Garmin quarter-turn tab and Varia receiver, PCB standoffs (M2),
# motor bulkhead, 3 LED holes (11.5 mm snap-in holders) and 2 push-button holes (7 mm) on the +Y side,
# IMU pod on the top face and a ToF (VL53L8CX) bevel on the foot's lower +Y edge for the height calibration.
# Places board, motor and a Varia dummy and runs light-weight clearance checks.
# Run inside FreeCAD:  exec(open("<this file>").read())
# Coordinates: X = rearward, Y = width (left/right), Z = up. Front face (seatpost side) at X = 0.
import os, math
import FreeCAD as App, Part, Sketcher

DIR = "/home/jackdaw/Documents/PlatformIO/Projects/prototype_1_BA/3D_design"
SRC = os.path.join(DIR, "gehäuse_board.FCStd")
OUT = os.path.join(DIR, "enclosure_v3.FCStd")
if os.path.exists(OUT):
    OUT = os.path.join(DIR, "enclosure_v3_new.FCStd")
V = App.Vector
C = Sketcher.Constraint
log = []

# ------------------------------------------------------------------ parameters
PARAMS = [
    ("Main enclosure (X = rearward, Y = width, Z = up)", None, None),
    ("D_main", 72, "depth of upper block front->rear"),
    ("H_total", 160, "total height (room for the top lid screws above the board)"),
    ("W_total", 56, "width (44 + room for the 3S battery behind the board)"),
    ("Foot_D", 24, "foot sticks out behind main block"),
    ("Foot_H", 44, "foot height"),
    ("Wall", 2.5, "wall thickness"),
    ("R_main", 10, "profile corner radius (top + bottom-front)"),
    ("R_foot", 5, "foot rear corner radius"),
    ("R_inner", 4, "concave radius foot/main"),
    ("R_side", 3, "edge radius around side faces (must stay > Wall; 3 leaves room for screw heads near the edge)"),
    ("Side lid (-Y side)", None, None),
    ("Lid_T", 4, "lid thickness from outer side face (split plane at Y = -W/2 + Lid_T)"),
    ("Insert_D", 4.0, "hole for M3 heat-set insert (M3 x 5.7)"),
    ("Insert_L", 6.5, "insert hole depth"),
    ("Screw_D", 3.4, "M3 clearance hole in lid"),
    ("Head_D", 6.2, "counterbore for M3 Torx cheese head (ISO 14580, 5.5 mm)"),
    ("Head_H", 2.0, "counterbore depth"),
    ("Board", None, None),
    ("PCB_Y", -3.0, "Y of the board's back face (component side faces the lid)"),
    ("USB-C opening (charging module at the board's front edge)", None, None),
    ("USB_Z", 68.6, "centre height of the USB-C port (module outline Z 58.6..78.6)"),
    ("USB_W", 13, "opening length along Z"),
    ("USB_H", 9, "opening height along Y (covers the module soldered flat or on headers)"),
    ("Battery 3S Li-ion pack, behind the board", None, None),
    ("Bat_L", 72, "length, cell axis vertical (wires leave at the top)"),
    ("Bat_W", 55.5, "width along X"),
    ("Bat_T", 19.5, "thickness along Y"),
    ("Bat_Z0", 56, "bottom of the battery"),
    ("Foam_T", 3, "foam pad between battery and wall (squeezed 0.5 by the clamp bars)"),
    ("Clamp_T", 3, "clamp bar thickness"),
    ("Clamp_X1", 20, "clamp bar 1 centre"), ("Clamp_X2", 47, "clamp bar 2 centre"),
    ("LEDs and buttons (+Y side, below the board)", None, None),
    ("LED_D", 11.5, "bore for 8 mm LED snap-in holder"),
    ("LED_Z", 27, "LED row height"),
    ("LED_X1", 12, ""), ("LED_X2", 26.5, ""), ("LED_X3", 41, ""),
    ("Btn_D", 7.2, "miniature push button, ~7 mm thread"),
    ("Btn_Z", 12, "button row height"),
    ("Btn_Power_X", 18, "power button (clear of the bottom-front screw column)"), ("Btn_Bedien_X", 33, "Bedientaster"),
    ("Motor", None, None),
    ("Motor_Z", 19.75, "motor axis height (encoder board must stay below the PCB at Z 37)"),
    ("Motor_Hole", 29.5, "rear wall hole for rotor (rotor 27.5)"),
    ("DFRobot FIT1035 motor + AS5600 encoder", None, None),
    ("Enc_Off", 6.5, "stator base -> encoder board (chip side); DFRobot factory spacers - measure yours"),
    ("Enc_SO_L", 6.0, "gold standoffs below the encoder board (mounting feet) - measure yours"),
    ("Mag_T", 2.5, "assumed diametric magnet on the shaft end (6 x 2.5); AS5600 wants 0.5..3 mm gap"),
    ("IMU (Adafruit BNO055 STEMMA QT, 25.4 x 20.3) in an OBS-style pod on the top face", None, None),
    ("IMU_X", 36.0, "pod / IMU centre along X (top face is flat from X 10 to 62)"),
    ("Pod_H", 7.0, "pod height above the top face"),
    ("Pod_L", 48.0, "pod top size along X"), ("Pod_W", 34.0, "pod top size along Y"),
    ("Pod_Taper", 15.0, "slope of the pod walls (deg from vertical)"),
    ("Pod_Lid_T", 2.0, "pod lid thickness (sits flush in a recess)"),
    ("ToF Pololu VL53L8CX carrier (12.7 x 22.9) in a bevel on the foot's lower +Y edge", None, None),
    ("Tof_Angle", 40, "view direction: degrees from straight down, tilted to the right (+Y), away from the wheel"),
    ("Tof_Bevel_W", 18, "width of the bevel face (pocket 13.1 + rims)"),
    ("Tof_X", 81.2, "board centre along X (pocket ends 0.7 mm before the foot's rear wall)"),
    ("Tof_Cover_T", 1.6, "flush printed cover with the sensor window, 2 x M2x8 countersunk"),
    ("Laser LFL650-5-12(9x20)60 on the rotor", None, None),
    ("Laser_D", 9.0, "brass body diameter"), ("Laser_L", 20.0, "body length"),
    ("Laser_Angle", 30, "default aim: degrees below horizontal, pointing to the left (-Y)"),
    ("LaserCable_Y", 19, "cable hole in the foot rear wall"), ("LaserCable_Z", 33, ""),
    ("LaserCable_D", 6.0, "cable hole (fit a 6 mm grommet)"),
    ("Garmin quarter-turn", None, None),
    ("Mount_Z", 106, "height of the seatpost tab (front); adapter must stay above the USB-C opening"),
    ("Rcv_Z", 128, "height of the Varia receiver (Varia top = Rcv_Z + Varia_TabTop)"),
    ("Varia_TabTop", 26, "Varia: tab centre below its top end (measured from photos)"),
    ("Tab_BaseD", 34, ""), ("Tab_BaseH", 3, ""), ("Tab_NeckD", 24.8, ""), ("Tab_NeckH", 1.6, ""),
    ("Tab_WingD", 28.6, ""), ("Tab_WingW", 10.8, ""), ("Tab_WingT", 1.5, ""),
    ("Rcv_OD", 36, ""), ("Rcv_CavD", 29.4, ""), ("Rcv_CavH", 1.7, "smaller = tighter Varia"),
    ("Rcv_OpenD", 25.4, ""), ("Rcv_LipT", 1.5, ""), ("Rcv_SlotW", 11.6, ""),
    ("WingAngle", 0, "0: tab wings vertical, receiver slots horizontal; 90 swaps"),
    ("Garmin adapters (separate printed parts)", None, None),
    ("Ad_T", 6, "adapter plate thickness (holds M3 x 5.7 heat-set inserts)"),
    ("Ad_W", 37, "adapter plate width (Y), fits the flat part of the 44 mm face"),
    ("Ad_R", 4, "adapter plate corner radius"),
    ("Tab_Ad_H", 56, "seatpost tab adapter height (Z)"),
    ("Rcv_Ad_H", 42, "Varia receiver adapter height (Z)"),
    ("Tab_ScrewY", 10, "tab adapter screws at +-Y (key path stays clear of the battery pocket)"),
    ("Rcv_ScrewY", 14, "receiver adapter screws at +-Y"),
    ("Tab_ScrewDZ", 22, "tab adapter screws at +-Z from the tab centre"),
    ("Rcv_ScrewDZ", 10, "receiver adapter screws at +-Z (keeps the L-key clear of the board standoff)"),
    ("CSK_D", 6.0, "countersink inside the housing wall for M3 countersunk Torx (ISO 14581, head 5.5)"),
]
p = {k: v for k, v, _ in PARAMS if v is not None}

doc = App.newDocument("enclosure_v3")
sh = doc.addObject("Spreadsheet::Sheet", "params")
for r, (name, val, note) in enumerate(PARAMS, start=1):
    sh.set("A%d" % r, name)
    if val is not None:
        sh.set("B%d" % r, str(val)); sh.setAlias("B%d" % r, name)
        if note:
            sh.set("C%d" % r, note)
doc.recompute()
doc.saveAs(OUT)

# ------------------------------------------------------------------ main body: outer shape
body = doc.addObject("PartDesign::Body", "Main"); body.Label = "gehäuse_main"
O = {f.Role: f for f in body.Origin.OriginFeatures}

def poly_sketch(name, pts):
    sk = body.newObject("Sketcher::SketchObject", name)
    sk.AttachmentSupport = [(O["XZ_Plane"], "")]; sk.MapMode = "FlatFace"
    for i in range(6):
        sk.addGeometry(Part.LineSegment(pts[i], pts[(i + 1) % 6]))
    for i in range(6):
        sk.addConstraint(C("Coincident", i, 2, (i + 1) % 6, 1))
    for i in (0, 2, 4):
        sk.addConstraint(C("Horizontal", i))
    for i in (1, 3, 5):
        sk.addConstraint(C("Vertical", i))
    return sk

D, Fd, Fh, H, W, t = p["D_main"], p["Foot_D"], p["Foot_H"], p["H_total"], p["W_total"], p["Wall"]
# profile corners are rounded inside the sketch (robust when dimensions change)
sk = poly_sketch("ProfileSketch", [V(0, 0, 0), V(D + Fd, 0, 0), V(D + Fd, Fh, 0), V(D, Fh, 0), V(D, H, 0), V(0, H, 0)])
oradii = {0: ("params.R_foot", p["R_foot"]), 1: ("params.R_foot", p["R_foot"]), 2: ("params.R_inner", p["R_inner"]),
          3: ("params.R_main", p["R_main"]), 4: ("params.R_main", p["R_main"]), 5: ("params.R_main", p["R_main"])}
oarc = {}
for i in range(6):
    oarc[i] = sk.GeometryCount
    sk.fillet(i, 2, oradii[i][1], True)
ex = {}
for i, a in oarc.items():
    ex[sk.addConstraint(C("Radius", a, oradii[i][1]))] = oradii[i][0]
sk.addConstraint(C("PointOnObject", -1, 1, 0))   # bottom line through origin
sk.addConstraint(C("PointOnObject", -1, 1, 5))   # front line through origin
ex[sk.addConstraint(C("DistanceX", -1, 1, 1, 1, D + Fd))] = "params.D_main + params.Foot_D"
ex[sk.addConstraint(C("DistanceY", -1, 1, 2, 1, Fh))] = "params.Foot_H"
ex[sk.addConstraint(C("DistanceX", -1, 1, 3, 1, D))] = "params.D_main"
ex[sk.addConstraint(C("DistanceY", -1, 1, 4, 1, H))] = "params.H_total"
for k, v in ex.items():
    sk.setExpression("Constraints[%d]" % k, v)
pad = body.newObject("PartDesign::Pad", "ProfilePad")
pad.Profile = sk; pad.Midplane = True; pad.setExpression("Length", "params.W_total")
sk.Visibility = False
doc.recompute()

def y_edges_at(shape, pts):
    out = []
    for i, e in enumerate(shape.Edges):
        if not isinstance(e.Curve, Part.Line):
            continue
        d = e.Vertexes[1].Point - e.Vertexes[0].Point
        if abs(d.x) > 1e-6 or abs(d.z) > 1e-6:
            continue
        q = e.Vertexes[0].Point
        if any(abs(q.x - x) < 1e-4 and abs(q.z - z) < 1e-4 for x, z in pts):
            out.append("Edge%d" % (i + 1))
    return out

def side_edges(shape, y):
    return ["Edge%d" % (i + 1) for i, e in enumerate(shape.Edges)
            if any(abs(e.BoundBox.YMin - s * y) < 1e-4 and abs(e.BoundBox.YMax - s * y) < 1e-4 for s in (1, -1))]

def fillet(name, base, edges, expr):
    f = body.newObject("PartDesign::Fillet", name)
    f.Base = (base, edges); f.setExpression("Radius", expr)
    doc.recompute()
    log.append("%s: %d edges, valid=%s" % (name, len(edges), f.Shape.isValid()))
    return f

f4 = fillet("Fillet_Side", pad, side_edges(pad.Shape, W / 2), "params.R_side")

# ------------------------------------------------------------------ cavity (rounded corners drawn in the sketch)
sk = poly_sketch("CavitySketch", [V(t, t, 0), V(D + Fd - t, t, 0), V(D + Fd - t, Fh - t, 0),
                                  V(D - t, Fh - t, 0), V(D - t, H - t, 0), V(t, H - t, 0)])
radii = {0: ("R_foot - params.Wall", p["R_foot"] - t), 1: ("R_foot - params.Wall", p["R_foot"] - t),
         2: ("R_inner + params.Wall", p["R_inner"] + t), 3: ("R_main - params.Wall", p["R_main"] - t),
         4: ("R_main - params.Wall", p["R_main"] - t), 5: ("R_main - params.Wall", p["R_main"] - t)}
arc = {}
for i in range(6):
    arc[i] = sk.GeometryCount
    sk.fillet(i, 2, radii[i][1], True)
ex = {}
for i, a in arc.items():
    ex[sk.addConstraint(C("Radius", a, radii[i][1]))] = "params." + radii[i][0]
ex[sk.addConstraint(C("DistanceY", -1, 1, 0, 1, t))] = "params.Wall"
ex[sk.addConstraint(C("DistanceX", -1, 1, 5, 1, t))] = "params.Wall"
ex[sk.addConstraint(C("DistanceX", -1, 1, 1, 1, D + Fd - t))] = "params.D_main + params.Foot_D - params.Wall"
ex[sk.addConstraint(C("DistanceY", -1, 1, 2, 1, Fh - t))] = "params.Foot_H - params.Wall"
ex[sk.addConstraint(C("DistanceX", -1, 1, 3, 1, D - t))] = "params.D_main - params.Wall"
ex[sk.addConstraint(C("DistanceY", -1, 1, 4, 1, H - t))] = "params.H_total - params.Wall"
for k, v in ex.items():
    sk.setExpression("Constraints[%d]" % k, v)
cav = body.newObject("PartDesign::Pocket", "Cavity")
cav.Profile = sk; cav.Type = "Length"; cav.Midplane = True
cav.setExpression("Length", "params.W_total - 2 * params.Wall")
sk.Visibility = False
doc.recompute()
hollow = fillet("CavFillet_Side", cav, side_edges(cav.Shape, W / 2 - t), "params.R_side - params.Wall")

def box(b, name, x, y, z, L, Wd, Hh, sub=False, zexpr=None, hexpr=None):
    org = {f.Role: f for f in b.Origin.OriginFeatures}
    f = b.newObject("PartDesign::" + ("SubtractiveBox" if sub else "AdditiveBox"), name)
    f.AttachmentSupport = [(org["XY_Plane"], "")]; f.MapMode = "ObjectXY"
    f.Length, f.Width, f.Height = L, Wd, Hh
    f.AttachmentOffset = App.Placement(V(x, y, z), App.Rotation())
    if zexpr:
        f.setExpression(".AttachmentOffset.Base.z", zexpr)
    if hexpr:
        f.setExpression("Height", hexpr)
    return f

def ycyl(b, name, x, z, dia, y1, y2, sub=True):
    """Cylinder along Y from y1 to y2 (y1 < y2). XZ plane normal is -Y, so it grows from y2 towards y1."""
    org = {f.Role: f for f in b.Origin.OriginFeatures}
    f = b.newObject("PartDesign::" + ("SubtractiveCylinder" if sub else "AdditiveCylinder"), name)
    f.AttachmentSupport = [(org["XZ_Plane"], "")]; f.MapMode = "ObjectXY"
    f.Radius = dia / 2; f.Height = y2 - y1
    f.AttachmentOffset = App.Placement(V(x, z, -y2), App.Rotation())
    return f

Y_SPLIT = -(W / 2 - p["Lid_T"])        # -18: housing keeps Y > Y_SPLIT, lid gets Y < Y_SPLIT
lidcut = box(body, "LidCut", -20, -40, -10, 140, 40 + Y_SPLIT, H + 20, sub=True)
lidcut.setExpression("Width", "(40 - params.W_total / 2 + params.Lid_T) * 1 mm")
doc.recompute()
doc.save()

# ------------------------------------------------------------------ Garmin interfaces as separate screw-on adapters
# Printed flat on their mounting face, so the quarter-turn features come out accurate and strong without supports,
# and a worn/broken one is reprinted in minutes instead of the whole housing (OpenBikeSensor-style mount adapter).
# Each adapter holds 4 M3 heat-set inserts, pressed in from its mounting face. M3x8 countersunk Torx screws
# (ISO 14581) go through the housing wall from the inside; their heads sit flush with the inner wall, so the board
# still slides in. Fit the adapters before the board: reach the screws with an L-key through the side opening.
PP = "params."

def make_lcs(b, name, reversed_, xexpr, height_param, angle_expr):
    org = {f.Role: f for f in b.Origin.OriginFeatures}
    l = b.newObject("PartDesign::CoordinateSystem", name)
    l.AttachmentSupport = [(org["YZ_Plane"], "")]; l.MapMode = "ObjectXY"; l.MapReversed = reversed_
    l.AttachmentOffset = App.Placement(V(0, 0, 0), App.Rotation(V(0, 0, 1), 1))
    l.setExpression(".AttachmentOffset.Base.y", "params.%s * 1 mm" % height_param)
    l.setExpression(".AttachmentOffset.Base.z", xexpr)
    l.setExpression(".AttachmentOffset.Rotation.Angle", angle_expr)
    l.Visibility = False
    return l

def on(b, kind, name, l):
    f = b.newObject("PartDesign::" + kind, name)
    f.AttachmentSupport = [(l, "")]; f.MapMode = "ObjectXY"
    return f

def xprim(b, kind, name, y, zexpr, x0, length, r1, r2=None):
    """cylinder/cone along +X starting at x0 (YZ plane: local x -> Y, local y -> Z, local z -> X)"""
    org = {f.Role: f for f in b.Origin.OriginFeatures}
    f = b.newObject("PartDesign::" + kind, name)
    f.AttachmentSupport = [(org["YZ_Plane"], "")]; f.MapMode = "ObjectXY"
    if "Cone" in kind:
        f.Radius1, f.Radius2 = r1, r2
    else:
        f.Radius = r1
    f.Height = length
    f.AttachmentOffset = App.Placement(V(y, 0, x0), App.Rotation())
    f.setExpression(".AttachmentOffset.Base.y", zexpr)
    return f

def plate(b, name, x0, zc_param, h_param):
    f = box(b, name, x0, -p["Ad_W"] / 2, p[zc_param] - p[h_param] / 2, p["Ad_T"], p["Ad_W"], p[h_param])
    f.setExpression(".AttachmentOffset.Base.z", "(params.%s - params.%s / 2) * 1 mm" % (zc_param, h_param))
    f.setExpression("Height", "params.%s * 1 mm" % h_param)
    doc.recompute()
    edges = ["Edge%d" % (i + 1) for i, e in enumerate(f.Shape.Edges)
             if isinstance(e.Curve, Part.Line) and abs((e.Vertexes[1].Point - e.Vertexes[0].Point).normalize().x) > 0.99]
    fl = b.newObject("PartDesign::Fillet", name + "_Round"); fl.Base = (f, edges); fl.setExpression("Radius", "params.Ad_R")
    doc.recompute()
    return fl

def screw_pattern(zc_param, dz_param, y_param):
    return [(sy * p[y_param], "(params.%s %s params.%s) * 1 mm" % (zc_param, "+" if sz > 0 else "-", dz_param),
             p[zc_param] + sz * p[dz_param]) for sy in (1, -1) for sz in (1, -1)]

TAB_SCREWS = screw_pattern("Mount_Z", "Tab_ScrewDZ", "Tab_ScrewY")
RCV_SCREWS = screw_pattern("Rcv_Z", "Rcv_ScrewDZ", "Rcv_ScrewY")
csk_h = (p["CSK_D"] - p["Screw_D"]) / 2

# --- seatpost tab adapter (front face, X <= 0)
tabad = doc.addObject("PartDesign::Body", "TabAdapter"); tabad.Label = "garmin_tab_adapter"
plate(tabad, "TabPlate", -p["Ad_T"], "Mount_Z", "Tab_Ad_H")
lt = make_lcs(tabad, "LCS_Tab", True, "params.Ad_T * 1 mm", "Mount_Z", "(params.WingAngle + 90) * 1 deg")  # z -> -X
f = on(tabad, "AdditiveCylinder", "Tab_Base", lt)
f.setExpression("Radius", PP + "Tab_BaseD * 1 mm / 2"); f.setExpression("Height", PP + "Tab_BaseH * 1 mm + 0.5 mm")
f.setExpression(".AttachmentOffset.Base.z", "-0.5 mm")
f = on(tabad, "AdditiveCylinder", "Tab_Wings", lt)
f.setExpression("Radius", PP + "Tab_WingD * 1 mm / 2"); f.setExpression("Height", PP + "Tab_WingT * 1 mm")
f.setExpression(".AttachmentOffset.Base.z", "(params.Tab_BaseH + params.Tab_NeckH) * 1 mm")
for i, side in enumerate(("+", "-")):
    f = on(tabad, "SubtractiveBox", "Tab_WingTrim%d" % (i + 1), lt)
    f.setExpression("Length", "(params.Tab_WingD + 4) * 1 mm")
    f.setExpression("Width", "(params.Tab_WingD / 2 + 2) * 1 mm")
    f.setExpression("Height", PP + "Tab_WingT * 1 mm")
    f.setExpression(".AttachmentOffset.Base.x", "-(params.Tab_WingD + 4) * 1 mm / 2")
    f.setExpression(".AttachmentOffset.Base.y", "params.Tab_WingW * 1 mm / 2" if side == "+"
                    else "-(params.Tab_WingW / 2 + params.Tab_WingD / 2 + 2) * 1 mm")
    f.setExpression(".AttachmentOffset.Base.z", "(params.Tab_BaseH + params.Tab_NeckH) * 1 mm")
f = on(tabad, "AdditiveCylinder", "Tab_Neck", lt)
f.setExpression("Radius", PP + "Tab_NeckD * 1 mm / 2"); f.setExpression("Height", "(params.Tab_NeckH + params.Tab_WingT) * 1 mm")
f.setExpression(".AttachmentOffset.Base.z", PP + "Tab_BaseH * 1 mm")
for i, (y, zexpr, _) in enumerate(TAB_SCREWS):
    xprim(tabad, "SubtractiveCylinder", "TabInsert%d" % (i + 1), y, zexpr, -p["Insert_L"] - 0.5, p["Insert_L"] + 1.0, p["Insert_D"] / 2)

# --- Varia receiver adapter (rear face of the main block, X >= D)
rcvad = doc.addObject("PartDesign::Body", "ReceiverAdapter"); rcvad.Label = "varia_receiver_adapter"
plate(rcvad, "RcvPlate", D, "Rcv_Z", "Rcv_Ad_H")
lr = make_lcs(rcvad, "LCS_Receiver", False, "(params.D_main + params.Ad_T) * 1 mm", "Rcv_Z", "params.WingAngle * 1 deg")
f = on(rcvad, "AdditiveCylinder", "Rcv_Boss", lr)
f.setExpression("Radius", PP + "Rcv_OD * 1 mm / 2"); f.setExpression("Height", "(params.Rcv_CavH + params.Rcv_LipT) * 1 mm + 0.5 mm")
f.setExpression(".AttachmentOffset.Base.z", "-0.5 mm")
f = on(rcvad, "SubtractiveCylinder", "Rcv_Cavity", lr)
f.setExpression("Radius", PP + "Rcv_CavD * 1 mm / 2"); f.setExpression("Height", PP + "Rcv_CavH * 1 mm")
f = on(rcvad, "SubtractiveCylinder", "Rcv_Opening", lr)
f.setExpression("Radius", PP + "Rcv_OpenD * 1 mm / 2"); f.setExpression("Height", "(params.Rcv_CavH + params.Rcv_LipT) * 1 mm + 1 mm")
f = on(rcvad, "SubtractiveBox", "Rcv_Slots", lr)
f.setExpression("Length", PP + "Rcv_CavD * 1 mm"); f.setExpression("Width", PP + "Rcv_SlotW * 1 mm")
f.setExpression("Height", "(params.Rcv_CavH + params.Rcv_LipT) * 1 mm + 1 mm")
f.setExpression(".AttachmentOffset.Base.x", "-" + PP + "Rcv_CavD * 1 mm / 2")
f.setExpression(".AttachmentOffset.Base.y", "-" + PP + "Rcv_SlotW * 1 mm / 2")
for i, (y, zexpr, _) in enumerate(RCV_SCREWS):
    xprim(rcvad, "SubtractiveCylinder", "RcvInsert%d" % (i + 1), y, zexpr, D - 0.5, p["Insert_L"] + 0.5, p["Insert_D"] / 2)

# --- matching holes in the housing: clearance hole + countersink on the inside of the wall
for i, (y, zexpr, _) in enumerate(TAB_SCREWS):
    xprim(body, "SubtractiveCylinder", "TabScrewHole%d" % (i + 1), y, zexpr, -1.0, t + 2.0, p["Screw_D"] / 2)
    xprim(body, "SubtractiveCone", "TabScrewCsk%d" % (i + 1), y, zexpr, t - csk_h, csk_h + 0.01, p["Screw_D"] / 2, p["CSK_D"] / 2)
for i, (y, zexpr, _) in enumerate(RCV_SCREWS):
    xprim(body, "SubtractiveCylinder", "RcvScrewHole%d" % (i + 1), y, zexpr, D - t - 1.0, t + 2.0, p["Screw_D"] / 2)
    xprim(body, "SubtractiveCone", "RcvScrewCsk%d" % (i + 1), y, zexpr, D - t - 0.01, csk_h + 0.01, p["CSK_D"] / 2, p["Screw_D"] / 2)
doc.recompute()

# ------------------------------------------------------------------ PCB standoffs (board: X 4..68, Y 8.45..10, Z 37..147)
# board is put in through the side opening and screwed onto 4 standoffs on the +Y wall (PCB holes 2.2 mm -> M2)
PCB_HOLES = [(8, 41), (64, 41), (8, 143), (64, 143)]
PY = p["PCB_Y"]
for i, (x, z) in enumerate(PCB_HOLES):
    ycyl(body, "PCB_Standoff%d" % (i + 1), x, z, 7.0, PY, W / 2 - t + 0.5, sub=False)
    ycyl(body, "PCB_Pilot%d" % (i + 1), x, z, 1.8, PY - 0.5, PY + 8.0)   # M2 self-tapping screw, 8 deep
# (motor wall is added in the motor section, it depends on the encoder stack)

# ------------------------------------------------------------------ bosses for M3 heat-set inserts (lid screws)
ZB_TOP, ZB_BOT, ZB_FOOT = H - t - 5.0, t + 5.0, Fh - t - 5.0
# Screws sit ~6.5 mm from the outer edge, OpenBikeSensor style: in the rounded corners (on the corner
# bisector), on the bottom edge next to the motor wall, and in the two rear corners of the foot.
# Nothing may stick out of the front/rear walls between Z 37..147, or the board could not go in.
# Each boss (D 9) reaches 0.5 mm into the walls, so it is merged with them like a corner column.
EDGE = 6.5
k = (p["R_main"] - EDGE) / math.sqrt(2)
# Where nothing is in the way the boss is a full-height column from the lid face to the opposite wall
# (OpenBikeSensor style, prints straight up from the bed); next to the encoder and the rotor it stays short.
FULL = (W / 2 - t + 0.5) - Y_SPLIT                         # split plane -> into the +Y wall
LID_SCREWS = [
    (p["R_main"] - k, H - p["R_main"] + k, FULL),         # top front corner: full column
    (D - p["R_main"] + k, H - p["R_main"] + k, FULL),     # top rear corner: full column
    (p["R_main"] - k, p["R_main"] - k, FULL),             # bottom front corner: full column (power button at X 18)
    (D - 10.0, EDGE, 8.0),                                # bottom edge, in front of the motor wall (short: encoder)
    (D + Fd - EDGE, EDGE, 6.5),                           # foot, bottom rear corner (short: rotor)
    (D + Fd - EDGE, Fh - EDGE, 6.5),                      # foot, top rear corner (short: rotor)
]
BOSS_D = 9.0
for i, (x, z, boss_l) in enumerate(LID_SCREWS):
    ycyl(body, "LidBoss%d" % (i + 1), x, z, BOSS_D, Y_SPLIT, Y_SPLIT + boss_l, sub=False)
    f = ycyl(body, "Insert_Hole%d" % (i + 1), x, z, p["Insert_D"], Y_SPLIT - 1, Y_SPLIT + p["Insert_L"])
    f.setExpression("Radius", "params.Insert_D * 1 mm / 2")
log.append("lid screws (X,Z): %s" % [(round(x, 2), round(z, 2)) for x, z, _ in LID_SCREWS])

# ------------------------------------------------------------------ LED and button holes through the +Y wall
def io_hole(name, xa, za, da):
    f = ycyl(body, name, 0, 0, 5, W / 2 - t - 1, W / 2 + 1)
    f.setExpression("Radius", "params.%s * 1 mm / 2" % da)
    f.setExpression(".AttachmentOffset.Base.x", "params.%s * 1 mm" % xa)
    f.setExpression(".AttachmentOffset.Base.y", "params.%s * 1 mm" % za)
    return f
for i in (1, 2, 3):
    io_hole("LED_Hole%d" % i, "LED_X%d" % i, "LED_Z", "LED_D")
io_hole("Btn_Power_Hole", "Btn_Power_X", "Btn_Z", "Btn_D")
io_hole("Btn_Bedien_Hole", "Btn_Bedien_X", "Btn_Z", "Btn_D")

# ------------------------------------------------------------------ USB-C opening in the front wall (stadium slot)
# The charging module sits on the component side at the board's front edge; its port centre is ~5 mm
# below the board's top face (flat) or ~7.5 mm (on headers) -> one slot covers both.
usb_y = PY - 1.55 - 3.5
usb_r = p["USB_H"] / 2
usb_half = p["USB_W"] / 2 - usb_r
org_ = {f.Role: f for f in body.Origin.OriginFeatures}
for i, dz in enumerate((-usb_half, usb_half)):
    f = body.newObject("PartDesign::SubtractiveCylinder", "USB_SlotEnd%d" % (i + 1))
    f.AttachmentSupport = [(org_["YZ_Plane"], "")]; f.MapMode = "ObjectXY"
    f.Radius = usb_r; f.Height = t + 2.0
    f.AttachmentOffset = App.Placement(V(usb_y, p["USB_Z"] + dz, -1.0), App.Rotation())
box(body, "USB_SlotMid", -1.0, usb_y - usb_r, p["USB_Z"] - usb_half, t + 2.0, p["USB_H"], 2 * usb_half, sub=True)
log.append("USB-C slot: centre Y %.2f Z %.1f, %.0f x %.0f mm" % (usb_y, p["USB_Z"], p["USB_W"], p["USB_H"]))

# ------------------------------------------------------------------ battery pocket on the +Y inner wall (behind the board)
yi_w = W / 2 - t                                   # inner face of the +Y wall
bat_x0 = 36.0 - p["Bat_W"] / 2                     # centred on the board (X 4..68)
bat_y1 = yi_w - p["Foam_T"] + 0.5                  # foam squeezed 0.5 mm
bat_y0 = bat_y1 - p["Bat_T"]                       # front face of the battery = seat of the clamp bars
bat_z0, bat_z1 = p["Bat_Z0"], p["Bat_Z0"] + p["Bat_L"]
gap, rib, rib_y0 = 0.4, 2.0, yi_w - 14.0           # ribs stand 14 mm off the wall
ox0, ox1 = bat_x0 - gap, bat_x0 + p["Bat_W"] + gap
oz0, oz1 = bat_z0 - gap, bat_z1 + gap
box(body, "BatRib_Front", ox0 - rib, rib_y0, oz0 - rib, rib, yi_w + 0.5 - rib_y0, oz1 - oz0 + 2 * rib)
box(body, "BatRib_Rear", ox1, rib_y0, oz0 - rib, rib, yi_w + 0.5 - rib_y0, oz1 - oz0 + 2 * rib)
box(body, "BatRib_Bottom", ox0 - rib, rib_y0, oz0 - rib, ox1 - ox0 + 2 * rib, yi_w + 0.5 - rib_y0, rib)
box(body, "BatRib_Top", ox0 - rib, rib_y0, oz1, ox1 - ox0 + 2 * rib, yi_w + 0.5 - rib_y0, rib)
box(body, "BatWireNotch", 30.0, rib_y0 - 0.5, oz1 - 0.5, 12.0, yi_w - rib_y0 + 0.4, rib + 1.0, sub=True)
# keep the L-key paths to the receiver-adapter screws free where they cross the rear rib
for (y, _, z) in RCV_SCREWS:
    if y > 0 and oz0 - rib < z < oz1 + rib:
        box(body, "BatRib_KeyNotch_%d" % int(z), ox1 - 0.5, rib_y0 - 0.5, z - 3.0, rib + 1.0, yi_w - rib_y0 + 0.4, 6.0, sub=True)
# 4 posts with M3 inserts for the 2 clamp bars, just above and below the battery
POST_Z = (oz0 - rib - 4.1, oz1 + rib + 4.1)
CLAMP_X = (p["Clamp_X1"], p["Clamp_X2"])
for i, x in enumerate(CLAMP_X):
    for j, z in enumerate(POST_Z):
        ycyl(body, "BatPost%d%d" % (i + 1, j + 1), x, z, 8.0, bat_y0, yi_w + 0.5, sub=False)
        ycyl(body, "BatPostInsert%d%d" % (i + 1, j + 1), x, z, p["Insert_D"], bat_y0 - 0.5, bat_y0 + p["Insert_L"])
log.append("battery: X %.2f..%.2f  Y %.2f..%.2f  Z %.1f..%.1f; posts Z %s" % (bat_x0, bat_x0 + p["Bat_W"], bat_y0, bat_y1, bat_z0, bat_z1, POST_Z))
doc.recompute()
doc.save()

# ------------------------------------------------------------------ board and motor from gehäuse_board
opened_src = False
src = None
for d in App.listDocuments().values():
    if d.FileName and os.path.abspath(d.FileName) == os.path.abspath(SRC):
        src = d
if src is None:
    src = App.openDocument(SRC, True); opened_src = True

MB = App.Matrix(0, -1, 0, -65.8,  0, 0, -1, p["PCB_Y"] + 5.0,  1, 0, 0, -56.8,  0, 0, 0, 1)  # board on edge, comp. side -Y
board = Part.getShape(src.getObject("_step_1")).copy(); board.transformShape(MB)
bo = doc.addObject("Part::Feature", "PCB_Board"); bo.Label = "PCB_STM32_board"; bo.Shape = board
# light-weight envelope of the board for clearance checks: one box per component + the PCB itself
env = []
for o in src.getObject("_step_1").Group:
    try:
        s = Part.getShape(src.getObject("_step_1"), o.Name + ".")
    except Exception:
        continue
    if s.isNull() or s.BoundBox.DiagonalLength < 1e-6 or s.BoundBox.DiagonalLength > 200:
        continue
    bb = s.BoundBox
    b = Part.makeBox(max(bb.XLength, 0.05), max(bb.YLength, 0.05), max(bb.ZLength, 0.05), V(bb.XMin, bb.YMin, bb.ZMin))
    b.transformShape(MB); env.append((o.Label, b))

# IMU board (Adafruit BNO055 STEMMA QT), exported from the IMU model in gehäuse_board to vendor_models/
# (own coordinates: PCB underside at z 0, centre at x = y = 0)
imu_shape = Part.read(os.path.join(DIR, "vendor_models", "Adafruit_BNO055_STEMMA_QT", "bno055_stemma_qt.brep"))
# pod geometry (Z levels): top face H, pod top H + Pod_H, lid recess Pod_Lid_T deep, 1 mm air above the IMU parts,
# IMU on 3 mm posts -> pocket floor. IMU axes = enclosure axes (board x -> +X rearward, y -> +Y, z -> up).
POD_TOP = H + p["Pod_H"]
POD_RECESS_Z = POD_TOP - p["Pod_Lid_T"]
IMU_POST_H = 3.0
IMU_PCB_Z = POD_RECESS_Z - 1.0 - 4.6 - 0.0             # board underside (parts reach 4.6 above it)
POD_FLOOR = IMU_PCB_Z - IMU_POST_H
imu_shape.translate(V(p["IMU_X"], 0, IMU_PCB_Z))
IMU_HOLES = [(sx * 10.16, sy * 7.62) for sx in (1, -1) for sy in (1, -1)]       # 4 x Ø2.5 (20.32 along X, 15.24 along Y)

if opened_src:
    App.closeDocument(src.Name)

# --- DFRobot FIT1035: 2208 motor + AS5600 encoder board (official STEP files, local copy in vendor_models/)
# Motor STEP frame: axis +y, stator base face y = 0, rotor top plate y 23..25, hub boss to y 26.5.
# Encoder STEP frame: board in XY (z 0..1.6), AS5600 chip on +z (faces the magnet), GH1.25 connectors on -z.
VEND = os.path.join(DIR, "vendor_models", "DFRobot_FIT1035_2208")
mot_raw = Part.read(os.path.join(VEND, "2208 Motor.STEP"))
enc_raw = Part.read(os.path.join(VEND, "AS5600 Magnetic Encoder.STEP"))
X_BASE = (D + Fd) + 1.0 - 25.0        # rotor plate face 1 mm behind the foot's rear wall -> bracket never rubs
MZ = p["Motor_Z"]
MM = App.Matrix(0, 1, 0, X_BASE,  1, 0, 0, 0,  0, 0, -1, MZ,  0, 0, 0, 1)   # motor local -> enclosure
ROT = -90.0                               # encoder connectors left/right (Y), motor wires up/right
def place(shape, extra=None):
    s = shape.copy()
    if extra is not None:
        s.transformShape(extra)
    s.transformShape(MM)
    s.rotate(V(0, 0, MZ), V(1, 0, 0), ROT)
    return s
# encoder frame -> motor local: x = xe, y = ze - Enc_Off - 1.6, z = -ye - 0.25 (slots line up with the 19 mm screws)
ME = App.Matrix(1, 0, 0, 0,  0, 0, 1, -(p["Enc_Off"] + 1.6),  0, -1, 0, -0.25,  0, 0, 0, 1)
rot_parts = [s for s in mot_raw.Solids if s.BoundBox.YMax > 20.0]          # rotor can + shaft
sta_parts = [s for s in mot_raw.Solids if s.BoundBox.YMax <= 20.0]         # base, windings, wire stubs
st = place(Part.makeCompound(sta_parts)); ro = place(Part.makeCompound(rot_parts))
en = place(enc_raw, ME)
so = doc.addObject("Part::Feature", "Motor_Stator"); so.Label = "2208_stator (DFRobot FIT1035)"; so.Shape = st
rr = doc.addObject("Part::Feature", "Motor_Rotor"); rr.Label = "2208_rotor (DFRobot FIT1035)"; rr.Shape = ro
eo = doc.addObject("Part::Feature", "Encoder_AS5600"); eo.Label = "AS5600_encoder_board (DFRobot)"; eo.Shape = en
# not in the STEP files: magnet (assumed 6 x Mag_T, on the shaft end), 2 spacers board->stator, 4 gold standoffs
mag = place(Part.makeCylinder(3.0, p["Mag_T"], V(0, -1.5 - p["Mag_T"], 0), V(0, 1, 0)))
SO_POS = [(-9.65, 9.65), (10.15, 9.65), (-9.65, -10.15), (10.15, -10.15)]     # outer ends of the diagonal slots
SP_POS = [(-9.5, -0.25), (9.5, -0.25)]                                        # 19 mm stator screws
def hexprism(af, h, x, y, z0):
    r = af / math.sqrt(3)
    pts_ = [V(x + r * math.cos(math.radians(30 + 60 * k)), y + r * math.sin(math.radians(30 + 60 * k)), z0) for k in range(7)]
    return Part.Face(Part.makePolygon(pts_)).extrude(V(0, 0, h))
gold = Part.makeCompound([place(hexprism(5.0, p["Enc_SO_L"], x, y, -p["Enc_SO_L"]), ME) for x, y in SO_POS])
silver = Part.makeCompound([place(Part.makeCylinder(2.5, p["Enc_Off"], V(x, y, 1.6)), ME) for x, y in SP_POS])
for nm, lbl, shp in (("Encoder_Magnet", "magnet_6x%.1f_assumed" % p["Mag_T"], mag), ("Encoder_Standoffs", "gold_standoffs", gold),
                     ("Encoder_Spacers", "spacers_board_to_motor", silver)):
    o_ = doc.addObject("Part::Feature", nm); o_.Label = lbl; o_.Shape = shp
# magnet -> chip air gap (motor local y): chip top at ze 3.32
chip_top = 3.32 - p["Enc_Off"] - 1.6
mag_face = -1.5 - p["Mag_T"]
ENC_GAP = mag_face - chip_top
log.append("AS5600 magnet->chip gap %.2f mm (needs 0.5..3, ideal ~1) with Enc_Off %.1f, Mag_T %.1f" % (ENC_GAP, p["Enc_Off"], p["Mag_T"]))
# standoff screw positions in the enclosure (Y, Z) and the motor wall face
holes = []
for x, y in SO_POS:
    q = place(Part.Vertex(V(x, y, -p["Enc_SO_L"])), ME).Point
    holes.append((round(q.y, 2), round(q.z, 2)))
X_BH = X_BASE - p["Enc_Off"] - 1.6 - p["Enc_SO_L"]       # rear face of the motor wall (standoff tips)
log.append("motor wall rear face X %.2f, standoff screws (Y,Z): %s" % (X_BH, holes))

def xcyl(name, y, z, dia, x0, length, sub=True):
    f = body.newObject("PartDesign::" + ("SubtractiveCylinder" if sub else "AdditiveCylinder"), name)
    f.AttachmentSupport = [(O["YZ_Plane"], "")]; f.MapMode = "ObjectXY"
    f.Radius = dia / 2; f.Height = length
    f.AttachmentOffset = App.Placement(V(y, z, x0), App.Rotation())
    return f

# motor wall: plate around the 4 standoffs (cables pass beside it), plus a low strip to the +Y wall for stiffness
BH_T = 3.0
box(body, "Motor_Wall", X_BH - BH_T, -14.0, 2.0, BH_T, 29.0, MZ + 14.0 - 2.0)
box(body, "Motor_WallStrip", X_BH - BH_T, 15.0 - 0.5, 2.0, BH_T, yi_w + 0.5 - 14.5, 6.0)
for i, (y, z) in enumerate(holes):
    xcyl("Motor_ScrewHole%d" % (i + 1), y, z, 3.2, X_BH - BH_T - 0.5, BH_T + 1.0)   # M2.5/M3 into the gold standoffs
f = xcyl("Motor_RotorHole", 0, p["Motor_Z"], p["Motor_Hole"], 92.5, 5.0)
f.setExpression("Radius", "params.Motor_Hole * 1 mm / 2")
f.setExpression(".AttachmentOffset.Base.y", "params.Motor_Z * 1 mm")
# laser cable into the foot, on the side the laser's cable end points to (fit a 6 mm rubber grommet)
f = xcyl("LaserCable_Hole", p["LaserCable_Y"], p["LaserCable_Z"], p["LaserCable_D"], D + Fd - t - 1.0, t + 2.0)
f.setExpression("Radius", "params.LaserCable_D * 1 mm / 2")
f.setExpression(".AttachmentOffset.Base.x", "params.LaserCable_Y * 1 mm")
f.setExpression(".AttachmentOffset.Base.y", "params.LaserCable_Z * 1 mm")

# ------------------------------------------------------------------ IMU pod on the top face (OpenBikeSensor GPS-pod style)
# Sloped pod, IMU inside on 4 posts, flush lid (separate part) with 2 x M3 button-head screws into heat-set inserts
# at one end, cable hole into the enclosure at the other end. ~13 cm from the motor magnets.
IX = p["IMU_X"]
def rrect_sketch(b, name, cx, cy, z, L, Wd, r):
    org = {f.Role: f for f in b.Origin.OriginFeatures}
    s = b.newObject("Sketcher::SketchObject", name)
    s.AttachmentSupport = [(org["XY_Plane"], "")]; s.MapMode = "FlatFace"
    s.AttachmentOffset = App.Placement(V(0, 0, z), App.Rotation())
    q = [V(cx - L / 2, cy - Wd / 2, 0), V(cx + L / 2, cy - Wd / 2, 0), V(cx + L / 2, cy + Wd / 2, 0), V(cx - L / 2, cy + Wd / 2, 0)]
    for i in range(4):
        s.addGeometry(Part.LineSegment(q[i], q[(i + 1) % 4]))
    for i in range(4):
        s.addConstraint(C("Coincident", i, 2, (i + 1) % 4, 1))
    for i in range(4):
        s.fillet(i, 2, r, True)
    s.Visibility = False
    return s
taper = math.radians(p["Pod_Taper"])
grow = p["Pod_H"] * math.tan(taper)
PL0, PW0 = p["Pod_L"] + 2 * grow, p["Pod_W"] + 2 * grow             # pod footprint on the top face
sk_pod = rrect_sketch(body, "PodSketch", IX, 0, H - 0.3, PL0, PW0, 4.0 + grow)
pod = body.newObject("PartDesign::Pad", "Pod")
pod.Profile = sk_pod; pod.Length = p["Pod_H"] + 0.3; pod.TaperAngle = -p["Pod_Taper"]
doc.recompute()
_top = [w.BoundBox for w in pod.Shape.slice(V(0, 0, 1), POD_TOP - 0.05)]
_bot = [w.BoundBox for w in pod.Shape.slice(V(0, 0, 1), H + 0.05) if w.BoundBox.ZMin > H - 1]
log.append("pod walls: X length near top %s, near the top face %s (should shrink upwards)"
           % ([round(b.XLength, 1) for b in _top], [round(b.XLength, 1) for b in _bot]))
REC_L, REC_W = p["Pod_L"] - 3.2, p["Pod_W"] - 3.2                  # lid recess: 1.6 mm rim at the top
CAV_L, CAV_W = REC_L - 3.6, REC_W - 3.6                            # pocket: 1.8 mm ledge for the lid
sk_rec = rrect_sketch(body, "PodRecessSketch", IX, 0, POD_TOP, REC_L, REC_W, 2.4)
pr = body.newObject("PartDesign::Pocket", "PodLidRecess"); pr.Profile = sk_rec; pr.Type = "Length"; pr.Length = p["Pod_Lid_T"]
# tub hanging under the top wall so the pocket can be deeper than the wall (board top edge is at Z 147)
box(body, "PodTub", IX - CAV_L / 2 - 2.0, -CAV_W / 2 - 2.0, POD_FLOOR - 2.0, CAV_L + 4.0, CAV_W + 4.0, (H - t + 0.5) - (POD_FLOOR - 2.0))
sk_cav = rrect_sketch(body, "PodCavitySketch", IX, 0, POD_RECESS_Z, CAV_L, CAV_W, 1.5)
pc = body.newObject("PartDesign::Pocket", "PodCavity"); pc.Profile = sk_cav; pc.Type = "Length"; pc.Length = POD_RECESS_Z - POD_FLOOR
doc.recompute()
# IMU posts (M2.5 self-tapping), lid-screw posts with M3 heat-set inserts (at the -X end), cable hole (+X end)
def zcyl(name, x, y, z0, h, r, sub):
    org = {f.Role: f for f in body.Origin.OriginFeatures}
    f = body.newObject("PartDesign::" + ("SubtractiveCylinder" if sub else "AdditiveCylinder"), name)
    f.AttachmentSupport = [(org["XY_Plane"], "")]; f.MapMode = "ObjectXY"
    f.Radius = r; f.Height = h
    f.AttachmentOffset = App.Placement(V(x, y, z0), App.Rotation())
    return f
for k, (dx, dy) in enumerate(IMU_HOLES):
    zcyl("IMU_Post%d" % (k + 1), IX + dx, dy, POD_FLOOR - 0.5, IMU_POST_H + 0.5, 2.5, False)
for k, (dx, dy) in enumerate(IMU_HOLES):
    zcyl("IMU_PostPilot%d" % (k + 1), IX + dx, dy, POD_FLOOR - 1.0, IMU_POST_H + 1.5, 1.1, True)
POD_SCREWS = [(IX - CAV_L / 2 + 3.6, sy * (CAV_W / 2 - 3.6)) for sy in (1, -1)]
for k, (x, y) in enumerate(POD_SCREWS):
    zcyl("PodLidPost%d" % (k + 1), x, y, POD_FLOOR - 0.5, POD_RECESS_Z - POD_FLOOR + 0.5, 3.75, False)
    zcyl("PodLidInsert%d" % (k + 1), x, y, POD_RECESS_Z - p["Insert_L"], p["Insert_L"] + 0.5, p["Insert_D"] / 2, True)
zcyl("PodCableHole", IX + CAV_L / 2 - 4.5, 0, POD_FLOOR - 3.0, 4.0, 3.0, True)
log.append("IMU pod: footprint %.1f x %.1f on the top face, top Z %.1f, pocket floor Z %.1f, IMU board at Z %.1f"
           % (PL0, PW0, POD_TOP, POD_FLOOR, IMU_PCB_Z))
doc.recompute()
doc.save()

# ------------------------------------------------------------------ ToF (Pololu VL53L8CX carrier) in a bevel on the foot's lower +Y edge
# Measures the laser height once on the mounted bike (bike upright): it looks down and to the right (+Y), past the
# rear wheel, Tof_Angle from straight down. The board sits in a pocket in the bevel, sensor side out, under a flush
# printed cover with a window; 2 x M2x8 countersunk screws go through cover and board holes into self-tapping pilots,
# so the sensor can be serviced from outside. Wires are soldered to the back of the 9-pin header and go through a
# slot in the pocket floor into the housing. The bevel runs out through the rear face; the motor stays 4 mm away.
# Board frame (TOF_PL): x = board width (12.7) up the bevel, y = board length (22.9) along +X, z = out of the bevel.
tb = math.radians(p["Tof_Angle"])
TV = V(0, math.cos(tb), math.sin(tb))              # up the bevel, towards the side face
TN = V(0, math.sin(tb), -math.cos(tb))             # bevel normal = view direction (down and to the right)
tw = p["Tof_Bevel_W"]
TC = V(p["Tof_X"], W / 2 - tw * math.cos(tb) / 2, tw * math.sin(tb) / 2)   # centre of the bevel face
TOF_PL = App.Placement(TC, App.Rotation(TV, V(1, 0, 0), TN, "ZXY"))
TOF_BW, TOF_BL, TOF_PCB = 12.7, 22.86, 1.016
TOF_GAP = 1.4                                      # cover underside -> PCB top (parts on the board are 1.2 tall)
TOF_FLOOR = -(p["Tof_Cover_T"] + TOF_GAP + TOF_PCB)   # pocket floor = back of the board
TOF_HOLES = [(10.16 - TOF_BW / 2, y - 11.43) for y in (2.54, 20.32)]   # board's 2 x M2 holes (upper edge)
PKT_W, PKT_L = TOF_BW + 0.4, TOF_BL + 0.4
TX0 = p["Tof_X"] - PKT_L / 2 - 2.4                 # front end of the bevel
lcs_t = body.newObject("PartDesign::CoordinateSystem", "LCS_ToF")
lcs_t.AttachmentSupport = [(O["XY_Plane"], "")]; lcs_t.MapMode = "ObjectXY"
lcs_t.AttachmentOffset = TOF_PL; lcs_t.Visibility = False
def tprim(kind, name, x, y, z, **kw):
    f = body.newObject("PartDesign::" + kind, name)
    f.AttachmentSupport = [(lcs_t, "")]; f.MapMode = "ObjectXY"
    for k_, v_ in kw.items():
        setattr(f, k_, v_)
    f.AttachmentOffset = App.Placement(V(x, y, z), App.Rotation())
    return f
# the bevel opens the cavity: a block behind it (inside the walls) and a cap at its front end close it again
yi_t = W / 2 - t
z_up = TC.z + math.tan(tb) * (yi_t - TC.y)         # bevel plane meets the inner side wall at this Z
y_lo = TC.y + (t - TC.z) / math.tan(tb)            # ... and the inner bottom wall at this Y
box(body, "ToF_FrontCap", TX0 - 2.5, y_lo - 1.0, t - 0.5, 3.0, yi_t + 0.5 - (y_lo - 1.0), z_up + 1.0 - (t - 0.5))
xin = tw / 2 - 1.5
tprim("AdditiveBox", "ToF_Block", -xin, TX0 - 0.5 - p["Tof_X"], TOF_FLOOR - 2.0,
      Length=2 * xin, Width=(D + Fd - t + 0.1) - (TX0 - 0.5), Height=-(TOF_FLOOR - 2.0) + 0.2)
tprim("SubtractiveBox", "ToF_Bevel", -30, TX0 - p["Tof_X"], 0, Length=60, Width=(D + Fd + 10) - TX0, Height=30)
tprim("SubtractiveBox", "ToF_Pocket", -PKT_W / 2, -PKT_L / 2, TOF_FLOOR, Length=PKT_W, Width=PKT_L, Height=-TOF_FLOOR + 1.0)
# wires from the 9-pin header (board x 1.27, lower edge) and room for the 4-pin row's solder joints (x 11.43)
tprim("SubtractiveBox", "ToF_WireSlot", -PKT_W / 2, -PKT_L / 2, TOF_FLOOR - 3.0, Length=PKT_W / 2 - 3.4, Width=PKT_L, Height=3.5)
tprim("SubtractiveBox", "ToF_PinRelief", 4.0, -6.0, TOF_FLOOR - 3.0, Length=PKT_W / 2 - 4.0, Width=9.5, Height=3.5)
for k_, (hx, hy) in enumerate(TOF_HOLES):
    tprim("AdditiveCylinder", "ToF_Boss%d" % (k_ + 1), hx, hy, -9.0, Radius=2.25, Height=(TOF_FLOOR - 2.0 + 0.3) + 9.0)
    tprim("SubtractiveCylinder", "ToF_Pilot%d" % (k_ + 1), hx, hy, -8.6, Radius=0.8, Height=TOF_FLOOR + 0.5 + 8.6)
log.append("ToF bevel: %.0f deg, face %.1f wide from X %.1f through the rear face, pocket floor %.2f deep, X %.1f..%.1f"
           % (p["Tof_Angle"], tw, TX0, -TOF_FLOOR, p["Tof_X"] - PKT_L / 2, p["Tof_X"] + PKT_L / 2))
doc.recompute()
doc.save()

# ------------------------------------------------------------------ side lid (-Y side)
cap = doc.addObject("PartDesign::Body", "Lid"); cap.Label = "gehäuse_lid"
binder = cap.newObject("PartDesign::SubShapeBinder", "LidBinder")
binder.Support = [(hollow, ("",))]; binder.Visibility = False
doc.recompute()
cap.BaseFeature = binder
doc.recompute()
keep = box(cap, "LidKeepOuter", -20, Y_SPLIT, -10, 140, W - p["Lid_T"] + 10, H + 20, sub=True)   # remove all with Y > split
keep.setExpression("Width", "(params.W_total - params.Lid_T + 10) * 1 mm")
keep.setExpression(".AttachmentOffset.Base.y", "-(params.W_total / 2 - params.Lid_T) * 1 mm")
yin = -(W / 2 - t)          # inner face of the lid plate (-19.5)
for i, (x, z, _) in enumerate(LID_SCREWS):
    ycyl(cap, "LidPad%d" % (i + 1), x, z, BOSS_D, yin - 0.5, Y_SPLIT, sub=False)
    f = ycyl(cap, "LidScrewHole%d" % (i + 1), x, z, p["Screw_D"], -W / 2 - 1, Y_SPLIT + 1)
    f.setExpression("Radius", "params.Screw_D * 1 mm / 2")
    f = ycyl(cap, "LidCounterbore%d" % (i + 1), x, z, p["Head_D"], -W / 2 - 1, -W / 2 + p["Head_H"])
    f.setExpression("Radius", "params.Head_D * 1 mm / 2")
# locating lip segments (reach 2 mm into the housing, 0.2 mm gap to its inner walls)
LIP_Y0, LIP_Y1 = yin - 0.1, Y_SPLIT + 2.0
def lip(name, x0, x1, z0, z1):
    box(cap, name, x0, LIP_Y0, z0, x1 - x0, LIP_Y1 - LIP_Y0, z1 - z0)
lip("Lip_Front", t + 0.2, t + 1.4, 15, H - 15)
lip("Lip_Rear", D - t - 1.4, D - t - 0.2, 55, H - 15)
lip("Lip_Top", 28, 44, H - t - 1.4, H - t - 0.2)
lip("Lip_Bottom", 22, 50, t + 0.2, t + 1.4)
lip("Lip_FootRear", D + Fd - t - 1.4, D + Fd - t - 0.2, 13, 31)
lip("Lip_FootBottom", 74, 83, t + 0.2, t + 1.4)
doc.recompute()
# view-only "open" copy of the lid. Do NOT move the Lid body itself: its binder is Relative and stays
# where the housing is, so the trim cut would keep the wrong slice. Move/show this link instead.
lk = doc.addObject("App::Link", "Lid_Exploded")
lk.Label = "gehäuse_lid_OPEN (view only)"
lk.LinkedObject = cap
lk.LinkPlacement = App.Placement(V(0, -40, 0), App.Rotation())
lk.Visibility = False
doc.recompute()

# ------------------------------------------------------------------ Varia RTL515 dummy (98.6 x 39.6 x 19.7 mm)
# tab is ~26 mm below the Varia's top end (measured from photos of the RTL515/516), ends almost fully round
x0 = p["D_main"] + p["Ad_T"] + p["Rcv_CavH"] + p["Rcv_LipT"]; zc_v = p["Rcv_Z"]
v_top = zc_v + p["Varia_TabTop"]
vb = Part.makeBox(19.7, 39.6, 98.6, V(x0, -19.8, v_top - 98.6))
vb = vb.makeFillet(18.5, [e for e in vb.Edges if abs((e.Vertexes[1].Point - e.Vertexes[0].Point).normalize().x) > 0.99])
vb = vb.makeFillet(3.0, [e for e in vb.Edges if any(abs(e.BoundBox.XMin - xx) < 1e-6 and abs(e.BoundBox.XMax - xx) < 1e-6 for xx in (x0, x0 + 19.7))])
neck = Part.makeCylinder(12.5, 3.1, V(x0, 0, zc_v), V(-1, 0, 0))
# trim box is longer than the disc in X: coplanar faces made this boolean unreliable (sometimes a full disc)
wings = Part.makeCylinder(14.4, 1.5, V(x0 - 1.6, 0, zc_v), V(-1, 0, 0)).common(Part.makeBox(2.5, 11.0, 40, V(x0 - 3.6, -5.5, zc_v - 20)))
varia = vb.fuse([neck, wings]).removeSplitter()
lens_c = V(x0 + 19.6, 0, v_top - 15.0)
lens = Part.makeCylinder(11, 0.6, lens_c, V(1, 0, 0)).cut(Part.makeCylinder(6.5, 0.6, lens_c, V(1, 0, 0)))
vo = doc.addObject("Part::Feature", "Varia_Dummy"); vo.Label = "Varia_RTL515_dummy"; vo.Shape = varia
lo = doc.addObject("Part::Feature", "Varia_Lens"); lo.Label = "Varia_light"; lo.Shape = lens
doc.recompute()

# ------------------------------------------------------------------ battery clamp bars (separate printed parts)
# Two flat bars across the battery's front face, each with 2 countersunk M3x8 into the post inserts.
# Print flat (big face down). Their back face sits on the posts at Y = bat_y0 and squeezes the foam 0.5 mm.
clamps = []
for i, x in enumerate(CLAMP_X):
    cb = doc.addObject("PartDesign::Body", "BatteryClamp%d" % (i + 1)); cb.Label = "battery_clamp_bar_%d" % (i + 1)
    zb0, zb1 = POST_Z[0] - 4.0, POST_Z[1] + 4.0
    bar = box(cb, "Bar", x - 5.0, bat_y0 - p["Clamp_T"], zb0, 10.0, p["Clamp_T"], zb1 - zb0)
    doc.recompute()
    ed = ["Edge%d" % (k + 1) for k, e in enumerate(bar.Shape.Edges)
          if isinstance(e.Curve, Part.Line) and abs((e.Vertexes[1].Point - e.Vertexes[0].Point).normalize().y) > 0.99]
    rf = cb.newObject("PartDesign::Fillet", "BarRound"); rf.Base = (bar, ed); rf.Radius = 4.0
    org_c = {f.Role: f for f in cb.Origin.OriginFeatures}
    for j, z in enumerate(POST_Z):
        ycyl(cb, "Hole%d" % (j + 1), x, z, p["Screw_D"], bat_y0 - p["Clamp_T"] - 1, bat_y0 + 1)
        cs = cb.newObject("PartDesign::SubtractiveCone", "Csk%d" % (j + 1))   # XZ plane: grows towards -Y
        cs.AttachmentSupport = [(org_c["XZ_Plane"], "")]; cs.MapMode = "ObjectXY"
        cs.Radius1 = p["Screw_D"] / 2; cs.Radius2 = p["CSK_D"] / 2; cs.Height = csk_h + 0.01
        cs.AttachmentOffset = App.Placement(V(x, z, -(bat_y0 - p["Clamp_T"] + csk_h)), App.Rotation())
    doc.recompute()
    clamps.append(cb)

# ------------------------------------------------------------------ battery stand-in (3S, 72 x 55.5 x 19.5) and foam pad
bt = Part.makeBox(p["Bat_W"], p["Bat_T"], p["Bat_L"], V(bat_x0, bat_y0, bat_z0))
bt = bt.makeFillet(8.5, [e for e in bt.Edges if abs((e.Vertexes[1].Point - e.Vertexes[0].Point).normalize().z) > 0.99])
bo_ = doc.addObject("Part::Feature", "Battery_3S"); bo_.Label = "battery_3S_72x55.5x19.5"; bo_.Shape = bt
fp = Part.makeBox(p["Bat_W"], yi_w - bat_y1, p["Bat_L"], V(bat_x0, bat_y1, bat_z0))
fo_ = doc.addObject("Part::Feature", "Battery_Foam"); fo_.Label = "foam_pad_3mm"; fo_.Shape = fp
doc.recompute()

# ------------------------------------------------------------------ laser bracket on the rotor (separate printed part)
# Hub on the rotor's top plate with an SK-style split clamp CENTRED on it: the laser lies across the motor axis and
# its middle sits on the axis, so it turns about its own centre (like the prototype). Turning the motor swings the
# beam in the plane across the bike. The 3 M2 hub screws (into the rotor's M2 holes on a 12 mm circle) sit under the
# clamp: screw them in through the 3 vertical access holes BEFORE inserting the laser. Then slide the laser in and
# tighten the M2 pinch screw (turn the laser in its bore first to set the line orientation). Print hub-face down.
ROTOR_FACE = X_BASE + 25.0                       # rotor top plate face (hub boss Ø9 rises 1.5 above it)
la = math.radians(p["Laser_Angle"])
d_las = V(0, -math.cos(la), -math.sin(la))       # local x: laser beam direction (left and down)
z_loc = V(1, 0, 0)                               # local z: away from the rotor
y_loc = z_loc.cross(d_las)                       # local y: towards the clamp
LCS_PL = App.Placement(V(ROTOR_FACE, 0, MZ), App.Rotation(d_las, y_loc, z_loc, "ZXY"))
LR, LBW = p["Laser_D"] / 2, 1.6                  # laser radius, clamp wall
CL_W = 2 * (LR + 0.05) + 2 * LBW                 # block width across the bore (12.4)
CL_Y0 = -CL_W / 2                                # block centred on the motor axis
LY = 0.0                                         # laser axis passes through the motor axis
LZ = 3.0 + LBW + LR + 0.05                       # laser axis height above the rotor face
CL_H = LZ + LR + 5.5                             # block height (ears for the pinch screw)
CL_L = 16.0                                      # clamp length along the laser (laser sticks out 2 mm each end)
br = doc.addObject("PartDesign::Body", "LaserBracket"); br.Label = "laser_bracket"
org_b = {f.Role: f for f in br.Origin.OriginFeatures}
lcs_b = br.newObject("PartDesign::CoordinateSystem", "LCS_Laser")
lcs_b.AttachmentSupport = [(org_b["XY_Plane"], "")]; lcs_b.MapMode = "ObjectXY"
lcs_b.AttachmentOffset = LCS_PL; lcs_b.Visibility = False
def bprim(kind, name, pl, **kw):
    f = br.newObject("PartDesign::" + kind, name)
    f.AttachmentSupport = [(lcs_b, "")]; f.MapMode = "ObjectXY"
    for k, v in kw.items():
        setattr(f, k, v)
    f.AttachmentOffset = pl
    return f
P_ = App.Placement; R0 = App.Rotation()
RX = App.Rotation(V(0, 1, 0), 90)                 # primitive z -> local x
RY = App.Rotation(V(1, 0, 0), -90)                # primitive z -> local y
# Hub screws at bracket angles 0/120/240 deg on the rotor's 12 mm M2 circle: these keep the pinch screw between the
# access holes. The rotor's angle is free (it is the servo position), so the rotor model is turned to match.
inv_pl = LCS_PL.inverse()
def rotor_m2_holes(shape):
    res = []
    for fc in shape.Faces:
        s_ = fc.Surface
        if isinstance(s_, Part.Cylinder) and abs(s_.Radius - 0.8) < 0.02 and abs(abs(s_.Axis.x) - 1) < 1e-3:
            c_ = inv_pl.multVec(s_.Center)
            if not any(math.hypot(c_.x - a, c_.y - b) < 0.5 for a, b in res):
                res.append((c_.x, c_.y))
    return res
h0 = rotor_m2_holes(ro)
if len(h0) != 3:
    raise RuntimeError("expected 3 M2 holes in the rotor top, found %s" % h0)
ang0 = min((math.degrees(math.atan2(b, a)) for a, b in h0), key=lambda g: abs(((g + 60) % 120) - 60))
delta = -(((ang0 + 60) % 120) - 60)                  # smallest turn that puts a hole at 0 deg
ro.rotate(V(0, 0, MZ), V(1, 0, 0), delta); rr.Shape = ro
M2_HOLES = rotor_m2_holes(ro)
log.append("rotor turned %.1f deg; M2 holes now at bracket angles %s" % (delta, sorted(round(math.degrees(math.atan2(b, a))) for a, b in M2_HOLES)))
# ---- star base (the prototype's idea): one arm per opening of the rotor's top plate, each arm drops in as a key.
# The rotor plate is 2 mm thick with 6 openings between its spokes; the keys follow each opening's real outline
# (sampled from the DFRobot STEP) with 0.3 mm clearance and go 1.5 mm deep -> the bracket centres itself and
# cannot twist; the 3 M2 screws hold it down.
can = max(ro.Solids, key=lambda s: s.Volume)
def solid_at(r, a_deg, depth=1.0):
    return can.isInside(LCS_PL.multVec(V(r * math.cos(math.radians(a_deg)), r * math.sin(math.radians(a_deg)), -depth)), 0.01, True)
hit = [solid_at(10.0, k) for k in range(360)]
windows = []
if not all(hit) and any(hit):
    k0 = hit.index(True); run = []
    for i in range(k0, k0 + 361):
        if not hit[i % 360]:
            run.append(i)
        elif run:
            windows.append(((run[0] + run[-1]) / 2.0) % 360); run = []
if len(windows) != 6:
    raise RuntimeError("expected 6 openings in the rotor top plate, found %s" % windows)
KEY_CLR, KEY_D, KEY_R0, KEY_R1 = 0.3, 1.5, 7.5, 12.5
# built directly in body coordinates (a BaseFeature ignores the Part::Feature's Placement)
Z_AX = LCS_PL.Rotation.multVec(V(0, 0, 1))
star = [Part.makeCylinder(8.6, 3.0, LCS_PL.Base, Z_AX)]   # centre disc for the 3 screws
for a_c in windows:
    lo_pts, hi_pts = [], []
    r = KEY_R0
    while r <= KEY_R1 + 1e-6:
        lo = a_c
        while not solid_at(r, lo - 0.25) and a_c - lo < 40: lo -= 0.25
        hi = a_c
        while not solid_at(r, hi + 0.25) and hi - a_c < 40: hi += 0.25
        m = math.degrees(KEY_CLR / r)
        lo, hi = lo + m, hi - m
        lo_pts.append(V(r * math.cos(math.radians(lo)), r * math.sin(math.radians(lo)), -KEY_D))
        hi_pts.append(V(r * math.cos(math.radians(hi)), r * math.sin(math.radians(hi)), -KEY_D))
        r += 0.5
    pts_g = [LCS_PL.multVec(q) for q in lo_pts + list(reversed(hi_pts)) + [lo_pts[0]]]
    poly = Part.makePolygon(pts_g)
    star.append(Part.Face(poly).extrude(Z_AX * (KEY_D + 3.0)))   # key (below the face) + arm (above), one piece
base_shape = star[0].fuse(star[1:]).removeSplitter()
basef = doc.addObject("Part::Feature", "LaserBracket_StarBase"); basef.Label = "laser_bracket_star_base (from rotor shape)"
basef.Shape = base_shape; basef.Visibility = False
br.BaseFeature = basef
log.append("rotor plate openings at bracket angles %s -> 6 key arms" % [round(a_) for a_ in windows])
bprim("AdditiveBox", "ClampBlock", P_(V(-CL_L / 2, CL_Y0, 0), R0), Length=CL_L, Width=CL_W, Height=CL_H)
bprim("SubtractiveCylinder", "BossRecess", P_(V(0, 0, -0.1), R0), Radius=4.8, Height=1.8)
ACC_R = 2.1                                          # Ø4.2 access holes for the M2 heads (Ø3.8) and a driver
# Only the 2 screws beside the laser are used: the one under the slit would need an access hole straight through
# the clamp onto the laser. The 6 key arms already stop the bracket from turning, 2 x M2 hold it down.
USED_HOLES = [(cx, cy) for cx, cy in M2_HOLES if abs(cy) > 1.0]
log.append("hub screws used: %d of 3 (%s)" % (len(USED_HOLES), [(round(a, 1), round(b, 1)) for a, b in USED_HOLES]))
for k, (cx, cy) in enumerate(USED_HOLES):
    bprim("SubtractiveCylinder", "HubScrew%d" % (k + 1), P_(V(cx, cy, -0.5), R0), Radius=1.1, Height=4.0)          # M2
    bprim("SubtractiveCylinder", "HubScrewAccess%d" % (k + 1), P_(V(cx, cy, 1.8), R0), Radius=ACC_R, Height=CL_H)  # head seat
bprim("SubtractiveCylinder", "LaserBore", P_(V(-CL_L / 2 - 1, LY, LZ), RX), Radius=LR + 0.05, Height=CL_L + 2)
bprim("SubtractiveBox", "ClampSlit", P_(V(-CL_L / 2 - 1, LY - 0.6, LZ), R0), Length=CL_L + 2, Width=1.2, Height=CL_H)
PZ = LZ + LR + 2.7                                # pinch screw height (above the bore)
PX = 4.0                                          # clear of the 2 access holes (which sit at x = -3)
bprim("SubtractiveCylinder", "PinchScrew", P_(V(PX, CL_Y0 - 1, PZ), RY), Radius=1.1, Height=CL_W + 2)           # M2
nut = bprim("SubtractivePrism", "PinchNut", P_(V(PX, CL_Y0 + CL_W - 2.0, PZ), RY), Polygon=6, Circumradius=2.45, Height=3.0)  # M2 nut
doc.recompute()
log.append("laser bracket valid=%s solids=%d; laser axis %.1f mm off the motor axis, %.1f mm above the rotor face"
           % (br.Shape.isValid(), len(br.Shape.Solids), LY, LZ))

# ------------------------------------------------------------------ laser LFL650-5-12(9x20)60 (brass, gold)
lb = Part.makeCylinder(LR, p["Laser_L"], V(-p["Laser_L"] / 2, LY, LZ), V(1, 0, 0))
lens = Part.makeCylinder(3.0, 0.3, V(p["Laser_L"] / 2 - 0.2, LY, LZ), V(1, 0, 0))
for nm, lbl, shp in (("Laser_Body", "laser_LFL650_9x20 (brass)", lb), ("Laser_Lens", "laser_aperture_6mm", lens)):
    s_ = shp.copy(); s_.Placement = LCS_PL.multiply(s_.Placement)
    o_ = doc.addObject("Part::Feature", nm); o_.Label = lbl; o_.Shape = s_
doc.recompute()

# ------------------------------------------------------------------ IMU pod lid (separate printed part) + IMU board
# Flush plate in the pod's recess (0.2 mm gap all round), 2 x M3 button-head (ISO 7380 Torx) into the inserts.
# Print flat, outer face up.
plid = doc.addObject("PartDesign::Body", "IMU_PodLid"); plid.Label = "imu_pod_lid"
sk_pl = rrect_sketch(plid, "PodLidSketch", IX, 0, POD_RECESS_Z, REC_L - 0.4, REC_W - 0.4, 2.2)
pl_pad = plid.newObject("PartDesign::Pad", "PodLidPlate"); pl_pad.Profile = sk_pl; pl_pad.Length = p["Pod_Lid_T"]
org_l = {f.Role: f for f in plid.Origin.OriginFeatures}
for k, (x, y) in enumerate(POD_SCREWS):
    f = plid.newObject("PartDesign::SubtractiveCylinder", "PodLidScrew%d" % (k + 1))
    f.AttachmentSupport = [(org_l["XY_Plane"], "")]; f.MapMode = "ObjectXY"
    f.Radius = p["Screw_D"] / 2; f.Height = p["Pod_Lid_T"] + 2
    f.AttachmentOffset = App.Placement(V(x, y, POD_RECESS_Z - 1), App.Rotation())
doc.recompute()
io_ = doc.addObject("Part::Feature", "IMU_BNO055"); io_.Label = "IMU_Adafruit_BNO055_STEMMA_QT"; io_.Shape = imu_shape
doc.recompute()

# ------------------------------------------------------------------ ToF board (Pololu VL53L8CX carrier) + its cover (separate printed part)
# Board model from gehäuse_board (own coordinates: PCB underside z 0, sensor module at the board centre).
tof_shape = Part.read(os.path.join(DIR, "vendor_models", "VL53L8CX", "vl53l8cx_breakout.brep"))
tof_shape.translate(V(-TOF_BW / 2, -11.43, TOF_FLOOR)); tof_shape.transformShape(TOF_PL.toMatrix())
to_ = doc.addObject("Part::Feature", "ToF_VL53L8CX"); to_.Label = "ToF_Pololu_VL53L8CX_carrier"; to_.Shape = tof_shape
# Cover: flush with the bevel, 2 pads press the board at its holes, window over the module (module + 0.3, opening
# 40 deg outwards above the module top). Print outer face down.
CT = p["Tof_Cover_T"]
cov = Part.makeBox(PKT_W - 0.3, PKT_L - 0.3, CT, V(-(PKT_W - 0.3) / 2, -(PKT_L - 0.3) / 2, -CT))
for hx, hy in TOF_HOLES:
    cov = cov.fuse(Part.makeCylinder(2.1, TOF_GAP + 0.01, V(hx, hy, -CT - TOF_GAP), V(0, 0, 1)))
wx0, wx1 = 3.12 - 0.3 - TOF_BW / 2, 9.55 + 0.3 - TOF_BW / 2
wy0, wy1 = 9.9 - 0.3 - 11.43, 12.96 + 0.3 - 11.43
TOF_MOD_TOP = TOF_FLOOR + 2.846
def wrect(z, m):
    return Part.makePolygon([V(wx0 - m, wy0 - m, z), V(wx1 + m, wy0 - m, z), V(wx1 + m, wy1 + m, z), V(wx0 - m, wy1 + m, z), V(wx0 - m, wy0 - m, z)])
win = Part.makeLoft([wrect(TOF_MOD_TOP, 0), wrect(0.3, (0.3 - TOF_MOD_TOP) * math.tan(math.radians(40)))], True)
win = win.fuse(Part.makeBox(wx1 - wx0, wy1 - wy0, TOF_MOD_TOP + CT + 0.2, V(wx0, wy0, -CT - 0.1)))
cov = cov.cut(win)
for hx, hy in TOF_HOLES:
    cov = cov.cut(Part.makeCylinder(1.1, 10.0, V(hx, hy, -5.0), V(0, 0, 1)))                 # M2 clearance
    cov = cov.cut(Part.makeCone(1.1, 2.05, 0.95, V(hx, hy, -0.95), V(0, 0, 1)))              # 90 deg countersink
    cov = cov.cut(Part.makeCylinder(2.05, 1.0, V(hx, hy, 0.0), V(0, 0, 1)))
cov = cov.removeSplitter(); cov.transformShape(TOF_PL.toMatrix())
tc_ = doc.addObject("Part::Feature", "ToF_Cover"); tc_.Label = "tof_cover (print)"; tc_.Shape = cov
doc.recompute()

# ------------------------------------------------------------------ visibility (also needed when built headless:
# without a GUI every object is saved hidden). Show each body's final feature and all reference parts.
for o in doc.Objects:
    if o.TypeId == "PartDesign::Body":
        for f in o.Group:
            if f.TypeId.startswith("PartDesign::") or f.TypeId.startswith("Sketcher::"):
                f.Visibility = False
        if o.Tip:
            o.Tip.Visibility = True
        o.Visibility = True
    elif o.TypeId == "Part::Feature":
        o.Visibility = True
for n in ("Lid_Exploded", "LaserBracket_StarBase"):
    if doc.getObject(n):
        doc.getObject(n).Visibility = False

# ------------------------------------------------------------------ colours (GUI only)
try:
    import FreeCADGui as Gui
    col = {"Main": (0.18, 0.18, 0.19), "Lid": (0.25, 0.25, 0.26), "PCB_Board": (0.1, 0.45, 0.2),
           "Motor_Stator": (0.6, 0.6, 0.62), "Motor_Rotor": (0.45, 0.45, 0.48),
           "Varia_Dummy": (0.08, 0.08, 0.08), "Varia_Lens": (0.9, 0.1, 0.1),
           "TabAdapter": (0.35, 0.35, 0.38), "ReceiverAdapter": (0.35, 0.35, 0.38),
           "BatteryClamp1": (0.35, 0.35, 0.38), "BatteryClamp2": (0.35, 0.35, 0.38),
           "Battery_3S": (0.15, 0.35, 0.85), "Battery_Foam": (0.9, 0.9, 0.85),
           "Encoder_AS5600": (0.1, 0.25, 0.65), "Encoder_Standoffs": (0.85, 0.68, 0.2), "Encoder_Spacers": (0.75, 0.75, 0.78),
           "Encoder_Magnet": (0.3, 0.3, 0.32), "LaserBracket": (0.92, 0.35, 0.55),
           "Laser_Body": (0.83, 0.66, 0.22), "Laser_Lens": (0.55, 0.05, 0.05),
           "IMU_PodLid": (0.95, 0.4, 0.15), "IMU_BNO055": (0.12, 0.12, 0.45),
           "ToF_Cover": (0.95, 0.4, 0.15), "ToF_VL53L8CX": (0.15, 0.3, 0.75)}
    for n, c in col.items():
        Gui.getDocument(doc.Name).getObject(n).ShapeColor = c
except Exception:
    pass
doc.save()

# ------------------------------------------------------------------ light-weight clearance checks
doc.recompute(None, True, True)    # make sure every body is up to date before measuring
main_s, cap_s = body.Shape, cap.Shape
log.append("errors: %s" % ([o.Name for o in doc.Objects if any(k in o.State for k in ("Invalid", "Error"))] or "none"))
log.append("main valid=%s solids=%d bb=%s" % (main_s.isValid(), len(main_s.Solids), main_s.BoundBox))
log.append("lid valid=%s solids=%d bb=%s" % (cap_s.isValid(), len(cap_s.Solids), cap_s.BoundBox))
log.append("main/lid overlap %.3f mm3" % main_s.common(cap_s).Volume)
hits = []
for name, b in env:
    for nm, s in (("main", main_s), ("lid", cap_s)):
        v = b.common(s).Volume
        if v > 0.01:
            hits.append("%s vs %s: %.2f mm3" % (name, nm, v))
log.append("board component boxes hitting housing: %s" % (hits or "none"))
mz = p["Motor_Z"]
rotor_env = Part.makeCylinder(13.75, ROTOR_FACE - (X_BASE + 7.5), V(X_BASE + 7.5, 0, mz), V(1, 0, 0))  # spinning can
stator_env = Part.makeCylinder(14.0, 7.5, V(X_BASE, 0, mz), V(1, 0, 0))
for nm, s in (("rotor", rotor_env), ("stator", stator_env)):
    log.append("%s envelope vs main %.3f mm3, min gap %.2f mm" % (nm, s.common(main_s).Volume, s.distToShape(main_s)[0]))
log.append("varia vs main %.3f mm3, vs lid %.3f mm3" % (varia.common(main_s).Volume, varia.common(cap_s).Volume))
log.append("rotor envelope vs lid: min gap %.2f mm" % rotor_env.distToShape(cap_s)[0])
tab_s, rcv_s = tabad.Shape, rcvad.Shape
log.append("tab adapter valid=%s solids=%d bb=%s" % (tab_s.isValid(), len(tab_s.Solids), tab_s.BoundBox))
log.append("receiver adapter valid=%s solids=%d bb=%s" % (rcv_s.isValid(), len(rcv_s.Solids), rcv_s.BoundBox))
log.append("adapter overlaps: tab/main %.3f, rcv/main %.3f, rcv/lid %.3f, varia/rcv %.3f mm3"
           % (tab_s.common(main_s).Volume, rcv_s.common(main_s).Volume, rcv_s.common(cap_s).Volume, varia.common(rcv_s).Volume))
# Varia tab in the receiver adapter: locked (upright) must be held by the lip, sideways must come free
vtab = doc.getObject("Varia_Dummy").Shape.common(Part.makeBox(3.1, 40, 40, V(x0 - 3.1, -20, zc_v - 20)))   # tab only
for ang, label in ((0, "locked"), (90, "sideways")):
    tt = vtab.copy(); tt.rotate(V(0, 0, zc_v), V(1, 0, 0), ang)
    seated = tt.common(rcv_s).Volume
    tt.translate(V(0.3, 0, 0))
    log.append("varia tab %s: seated overlap %.3f, pulled 0.3 mm overlap %.2f mm3" % (label, seated, tt.common(rcv_s).Volume))
# L-key paths to the adapter screws (from the inside, board not yet fitted): must not hit the housing
blocked = []
for (y, _, z) in TAB_SCREWS:
    if Part.makeBox(17.0, 3.0, 3.0, V(t + 0.05, y - 1.5, z - 1.5)).common(main_s).Volume > 0.01: blocked.append(("tab", y, z))
for (y, _, z) in RCV_SCREWS:
    if Part.makeBox(17.0, 3.0, 3.0, V(D - t - 17.05, y - 1.5, z - 1.5)).common(main_s).Volume > 0.01: blocked.append(("rcv", y, z))
log.append("adapter screw key paths blocked: %s" % (blocked or "none"))
# IMU pod, its lid and the IMU board
pl_s = plid.Shape
log.append("IMU pod lid valid=%s solids=%d; lid vs main %.3f, IMU vs main %.3f, IMU vs pod lid %.3f mm3"
           % (pl_s.isValid(), len(pl_s.Solids), pl_s.common(main_s).Volume, imu_shape.common(main_s).Volume,
              imu_shape.common(pl_s).Volume))
log.append("IMU gaps: to pod (posts touch) %.2f, to pod lid %.2f; pod tub to board parts %.2f; IMU to motor rotor %.1f mm"
           % (imu_shape.distToShape(main_s)[0], imu_shape.distToShape(pl_s)[0],
              min(main_s.common(Part.makeBox(200, 200, 20, V(-50, -100, H - t - 12))).distToShape(b)[0] for _, b in env),
              imu_shape.distToShape(ro)[0]))
# full-height screw columns: clearances to the board, its standoffs and the buttons
for i, (x, z, L) in enumerate(LID_SCREWS):
    if L > 20:
        col = Part.makeCylinder(BOSS_D / 2, L, V(x, Y_SPLIT, z), V(0, 1, 0))
        pcb_gap = min(col.distToShape(b)[0] for _, b in env)
        log.append("screw column %d at (X %.1f, Z %.1f): gap to board parts %.2f mm" % (i + 1, x, z, pcb_gap))
# battery, foam and clamp bars
log.append("battery vs main %.3f, vs lid %.3f mm3; foam vs main %.3f mm3"
           % (bt.common(main_s).Volume, bt.common(cap_s).Volume, fp.common(main_s).Volume))
for cb in clamps:
    cs_ = cb.Shape
    pcb_hits = [n for n, b in env if cs_.common(b).Volume > 0.01]
    log.append("%s: valid=%s, vs main %.3f, vs battery %.3f mm3, gap to board parts %.2f mm, hits %s"
               % (cb.Name, cs_.isValid(), cs_.common(main_s).Volume, cs_.common(bt).Volume,
                  min(cs_.distToShape(b)[0] for _, b in env), pcb_hits or "none"))
log.append("battery gap to board parts %.2f mm" % min(bt.distToShape(b)[0] for _, b in env))
# board still slides in through the side opening?
sweep = Part.makeBox(64, PY + W / 2 + 2, 110, V(4, -W / 2 - 2, 37))
log.append("board insertion path blocked by housing: %.3f mm3" % sweep.common(main_s).Volume)
# motor + encoder
log.append("encoder board vs main %.3f, vs board parts %s; gap encoder -> PCB %.2f mm; standoffs vs main %.3f, spacers vs main %.3f"
           % (en.common(main_s).Volume, [n for n, b in env if en.common(b).Volume > 0.01] or "none",
              min(en.distToShape(b)[0] for _, b in env), gold.common(main_s).Volume, silver.common(main_s).Volume))
log.append("motor stator vs main %.3f, rotor vs main %.3f mm3; rotor gap to rear-wall hole %.2f mm"
           % (st.common(main_s).Volume, ro.common(main_s).Volume, rotor_env.distToShape(main_s)[0]))
# laser + bracket over its swing (rotations about the motor axis), against housing, lid and Varia
lz = Part.makeCompound([br.Shape, doc.getObject("Laser_Body").Shape])
worst = (99.0, None)
for a_ in range(-120, 121, 15):
    s_ = lz.copy(); s_.rotate(V(0, 0, MZ), V(1, 0, 0), a_)
    for nm, tgt in (("housing", main_s), ("lid", cap_s), ("varia", varia), ("tab adapter", tab_s)):
        dd = s_.distToShape(tgt)[0]
        if dd < worst[0]:
            worst = (dd, "%s at %+d deg" % (nm, a_))
log.append("laser+bracket swing -120..+120 deg: closest %.2f mm (%s)" % worst)
# bracket screw holes must sit over the rotor's M2 holes: no rotor material inside the bracket's Ø2.2 holes
rot_top = ro.common(Part.makeCylinder(14.0, 2.0, V(ROTOR_FACE - 2.0, 0, MZ), V(1, 0, 0)))
log.append("bracket vs rotor overlap %.3f mm3; rotor M2 holes under bracket holes: %s"
           % (br.Shape.common(ro).Volume, "yes" if Part.makeCompound([Part.makeCylinder(0.7, 1.5, LCS_PL.multVec(V(a, b, -1.6)), V(1, 0, 0)) for a, b in USED_HOLES]).common(rot_top).Volume < 0.01 else "NO"))
log.append("laser cable hole at Y %.0f Z %.0f, %.1f mm from the motor axis" % (p["LaserCable_Y"], p["LaserCable_Z"], math.hypot(p["LaserCable_Y"], p["LaserCable_Z"] - MZ)))
# USB-C slot clear of the tab adapter?
log.append("USB slot top Z %.1f, tab adapter bottom Z %.1f" % (p["USB_Z"] + p["USB_W"] / 2, p["Mount_Z"] - p["Tab_Ad_H"] / 2))
# keep-out volumes inside the housing: LED holder + LED (15 deep), button body (20 deep), hex/Torx key to motor screws
yi = W / 2 - t
ko = []
for i in (1, 2, 3):
    ko.append(("LED%d" % i, Part.makeCylinder(6.5, 15.0, V(p["LED_X%d" % i], yi - 0.01, p["LED_Z"]), V(0, -1, 0))))
for nm in ("Btn_Power_X", "Btn_Bedien_X"):
    ko.append((nm[:-2], Part.makeCylinder(4.5, 20.0, V(p[nm], yi - 0.01, p["Btn_Z"]), V(0, -1, 0))))
key = [Part.makeBox(20.0, 6.0, 6.0, V(X_BH - BH_T - 20.0, y - 3.0, z - 3.0)) for y, z in holes]
pcb_env = [b for n, b in env if n == "PCB_STM32_custom_board_PCB"][0]
for nm, s in ko:
    probs = []
    if s.common(main_s).Volume > 0.01: probs.append("housing")
    if s.common(pcb_env).Volume > 0.01 or any(s.common(b).Volume > 0.01 for _, b in env): probs.append("board")
    if s.common(stator_env).Volume > 0.01 or s.common(rotor_env).Volume > 0.01: probs.append("motor")
    if any(s.common(k).Volume > 0.01 for k in key): probs.append("motor-screw key path")
    log.append("keep-out %-10s %s" % (nm, ", ".join(probs) or "clear"))
# ToF: fit, clearances to the motor/encoder, and a free field of view (45 x 45 deg + margin) over 1.2 m
tof_s, tcov_s = tof_shape, cov
log.append("ToF board vs main %.3f, vs cover %.3f; cover vs main %.3f mm3; cover valid=%s solids=%d"
           % (tof_s.common(main_s).Volume, tof_s.common(tcov_s).Volume, tcov_s.common(main_s).Volume,
              tcov_s.isValid(), len(tcov_s.Solids)))
reg = Part.makeBox(tw, (D + Fd - t - 0.1) - (TX0 - 3.0), 10.0, V(-tw / 2, TX0 - 3.0 - p["Tof_X"], -9.5))   # ToF parts, not the rear wall
reg.transformShape(TOF_PL.toMatrix())
tof_part = main_s.common(reg)
log.append("ToF block/bosses: gap to rotor env %.2f, stator env %.2f, encoder board %.2f, standoffs %.2f mm"
           % (tof_part.distToShape(rotor_env)[0], tof_part.distToShape(stator_env)[0], tof_part.distToShape(en)[0],
              tof_part.distToShape(gold)[0]))
TOF_OPT_L = V(6.63 - TOF_BW / 2, 11.41 - 11.43, TOF_FLOOR + 2.794)   # receiver aperture centre (board frame)
def fov_pyramid(half_deg, L):
    tt = math.tan(math.radians(half_deg)) * L
    a = TOF_PL.multVec(TOF_OPT_L)
    cs = [TOF_PL.multVec(TOF_OPT_L + V(sx * tt, sy * tt, L)) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    fs = [Part.Face(Part.makePolygon([a, cs[i], cs[(i + 1) % 4], a])) for i in range(4)]
    fs.append(Part.Face(Part.makePolygon(cs + [cs[0]])))
    return Part.Solid(Part.Shell(fs))
fov = fov_pyramid(24.0, 1200.0)
fov_hits = []
for nm, tgt in (("housing", main_s), ("lid", cap_s), ("cover", tcov_s), ("tab adapter", tab_s), ("receiver adapter", rcv_s),
                ("varia", varia), ("rotor", ro)):
    vv = fov.common(tgt).Volume
    if vv > 0.001:
        fov_hits.append("%s %.3f mm3" % (nm, vv))
for a_ in range(-120, 121, 30):
    s_ = lz.copy(); s_.rotate(V(0, 0, MZ), V(1, 0, 0), a_)
    if fov.common(s_).Volume > 0.001:
        fov_hits.append("laser/bracket at %+d deg" % a_)
log.append("ToF field of view (2 x 24 deg) blocked by: %s" % (fov_hits or "nothing"))
S_ = TOF_PL.multVec(TOF_OPT_L); A_ = V(ROTOR_FACE + LZ, 0, MZ)
log.append("ToF optical centre X %.2f Y %.2f Z %.2f, view dir (Y %.3f, Z %.3f); laser pivot on the motor axis X %.2f Y 0 Z %.2f;"
           " sensor -> pivot dX %.2f dY %.2f dZ %.2f mm" % (S_.x, S_.y, S_.z, TN.y, TN.z, A_.x, A_.z, A_.x - S_.x, A_.y - S_.y, A_.z - S_.z))
log.append("saved: %s" % OUT)
print("\n".join(log))
