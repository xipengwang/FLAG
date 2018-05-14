"use strict";

// default camera controls

// BUG? receives coordinates relative to canvas, not relative to
// layer.
function makeComputeManipulationPointXYPlane(z)
{
    return makeComputeManipulationPointPlane(0, 0, 1, z);
}

// pick the point nearest the plane Ax+By+Cz=D
function makeComputeManipulationPointPlane(A, B, C, D)
{
    return function(layer, x, y) {
        // start of ray
        var r0 = gluUnproject([ x, layer.viewport[3] - y, 0],
                              layer.V, layer.P, layer.viewport);

        // direction of ray
        var r1 = gluUnproject([ x, layer.viewport[3] - y, 1],
                              layer.V, layer.P, layer.viewport);

        var dir = vec_subtract(r1, r0);

        // for every lambda=1 we move in direction 'dir', how much does the RHS change?
        var dirdot = vec_dotproduct(dir, [A, B, C]);

        // how much change do we need to achieve on the RHS?
        var dist0 = D - vec_dotproduct(r0, [A, B, C]);

        // compute how far we should go.
        var lambda = dist0 / dirdot;

        var p = mat_create(4, 1, [ r0[0]+lambda*dir[0],
                                   r0[1]+lambda*dir[1],
                                   r0[2]+lambda*dir[2],
                                   1 ]);

        return p;
    }
}

// this function operates in GL y coordinates (Y increases going up).
function computePanJacobian(xyz, P, viewport, eye, c, up, dir1, dir2)
{
    var eps = 0.0001;

    var M0 = elu_to_matrix(eye, c, up);
    var w0 = gluProject(xyz, M0, P, viewport);

    var M1 = elu_to_matrix(vec_add(eye, vec_scale(dir1, eps)),
                           vec_add(c, vec_scale(dir1, eps)),
                           up);
    var w1 = gluProject(xyz, M1, P, viewport);

    var M2 = elu_to_matrix(vec_add(eye, vec_scale(dir2, eps)),
                           vec_add(c, vec_scale(dir2, eps)),
                           up);
    var w2 = gluProject(xyz, M2, P, viewport);

    return mat_create(2, 2, [ (w1[0]-w0[0])/eps, (w2[0]-w0[0])/eps,
                              (w1[1]-w0[1])/eps, (w2[1]-w0[1])/eps ]);
}

// this function operates in GL y coordinates (Y increases going up).
function windowSpacePanTo(xyz, winx, winy, preservez, P, viewport, eye, lookat, up)
{
    var mv = mat_create(3, 1, [ 0, 0, 0]);
    eye = vec_copy(eye);
    lookat = vec_copy(lookat);
    up = vec_copy(up);

    for (var iter = 0; iter < 100; iter++) {
        var M = elu_to_matrix(eye, lookat, up);

        var winpos0 = gluProject(xyz, M, P, viewport);

        var err = mat_create(2, 1, [ winx - winpos0[0], winy - winpos0[1] ]);

        var lookVec = vec_normalize(vec_subtract(lookat, eye));
        var left = vec_crossproduct(up, lookVec);

        var dir1 = vec_copy(up);
        var dir2 = vec_copy(left);

        if (false) { // 2.5D  (if not in 2.5D, this can cause a NaN)
            dir1[2] = 0;
            dir2[2] = 0;
        }

        var J = computePanJacobian(xyz, P, viewport, eye, lookat, up, dir1, dir2);

        if (preservez) {
            dir1[2] = 0;
            dir2[2] = 0;
        }

        var weights = mat_multiply(mat_inverse(J), err);

        var dx = mat_create(3, 1, [ dir1[0]*weights[0] + dir2[0]*weights[1],
                                    dir1[1]*weights[0] + dir2[1]*weights[1],
                                    dir1[2]*weights[0] + dir2[2]*weights[1] ]);

        eye = vec_add(eye, dx);
        lookat = vec_add(lookat, dx);

        mv = vec_add(mv, dx);

        if (vec_magnitude(dx) < .001)
            break;
    }

    return mv;
}

////////////////////////////////////////

function DefaultCameraControls(layer)
{
    this.layer = layer;
    this.touchManipulationPoints = new Object();
    this.lastRotateX = 0;
    this.lastRotateY = 0;
    this.interfaceMode = "3D";
    this.cameraControlMask = 7; // ZOOM(1) + PAN(2) + ROTATE(4)
    this.setuporder = 0;
    this.eventorder = 0;
    this.mouseDown = false;
    this.touchDown = false;
    this.lastTouchCenter = null;
    this.lastTouchDist = 0;

    this.computeManipulationPoint = makeComputeManipulationPointXYPlane(0);

    this.animateMS = 200;
    this.touchAnimateMS = 150;

    var time = new Date().getTime();

    this.eluTime0 = time;
    this.eluTime1 = time;
    this.eye0 = [ 0, 0, 100 ];
    this.eye1 = this.eye0;
    this.eye0deriv = [ 0, 0, 0 ];
    this.lookat0 = [ 0, 0, 0 ];
    this.lookat0deriv = [ 0, 0, 0 ];
    this.lookat1 = this.lookat0;
    this.up0 = [ 0, 1, 0 ];
    this.up0deriv = [ 0, 0, 0 ];
    this.up1 = this.up0;

    this.backgroundTime0 = time;
    this.backgroundTime1 = time;
    this.background0 = [ .1, .1, .1, 1 ];
    this.background1 = this.background0;
    this.background0deriv = [ 0, 0, 0, 0 ];
}

// X0  a vector containing the state at time t0
// X1  a vector containing the desired state at t1
// X0d a vector containing the derivative of the state at time 0
// t0  the time corresponding to X0
// t1  the time (in the future) corresponding to X1
// t   the current time.

// (note X1d is implicitly zero!)
function interpolate(X0, X0d, X1, t0, t1, t)
{
    // after interpolation window?
    if (t >= t1)
        return { Y: X1, Yderiv: mat_create(X0.length, 1) };

    // map t0 to be time 0. scale to seconds. Watch out for conditioning.
    t1 = (t1 - t0) / 1000;
    t = (t - t0) / 1000;

    if (false) {
        // linear interpolation. (ignoring X0d.)
        var alpha = t / t1;

        var Y = mat_create(X0.length, 1);
        var Yderiv = mat_create(X0.length, 1);

        for (var i = 0; i < X0.length; i++) {
            Y[i] = (1.0 - alpha) * X0[i] + alpha * X1[i];
            Yderiv[i] = 0;
        }

        return { Y: Y, Yderiv: Yderiv };

    } else {
        // quadratic interpolation
        var A = mat_create(2, 2, [ t1*t1, t1*t1*t1, 2*t1, 3*t1*t1 ]);

        var A = mat_create(2, 2, [ t1*t1, t1*t1*t1, 2*t1, 3*t1*t1 ]);
        var Ainv = mat_inverse(A);

        var Y = mat_create(X0.length, 1);
        var Yderiv = mat_create(X0.length, 1);

        for (var i = 0; i < X0.length; i++) {
            var B = mat_create(2, 1, [ X1[i]-X0[i]-X0d[i]*t1, -X0d[i] ]);
            var sol = mat_multiply(Ainv, B);

            Y[i]      = X0[i] + X0d[i]*t + sol[0]*t*t + sol[1]*t*t*t;
            Yderiv[i] = X0d[i] + 2*sol[0]*t + 3*sol[1]*t*t;
        }

        return { Y: Y, Yderiv: Yderiv };
    }
}

DefaultCameraControls.prototype.gotoBackground = function(rgba, dtime)
{
    var time = new Date().getTime();

    var interp = interpolate(this.background0, this.background0deriv, this.background1, this.backgroundTime0, this.backgroundTime1, time);
    this.background0  = interp.Y;
    this.background0deriv = interp.Yderiv;
    this.background1 = rgba;
    this.backgroundTime0 = time;
    this.backgroundTime1 = time + dtime;

    this.layer.vc.requestContinuousRedraw(this.layer.name+":camera-rgba");
}

DefaultCameraControls.prototype.fixup = function(eye, lookat, up)
{
    switch (this.interfaceMode)
    {
        // no roll allowed
        case "2.5D":
        {
            // 2.5D mode
            var original_dist = vec_distance(eye, lookat);

            // always looking at z=0 (XXX use manip point?)
            lookat[2] = 0;

            var lookdir = vec_normalize(vec_subtract(lookat, eye));
            // make sure horizon is level
            var left = vec_crossproduct(up, lookdir);
            left[2] = 0;
            left = vec_normalize(left);
            up = vec_normalize(vec_crossproduct(lookdir, left));

            var dir = vec_crossproduct(up, left);

            eye = vec_add(lookat, vec_scale(dir, original_dist));

            break;
        }

        // locked straight down, no rotation
        case "2F": {
            // no rotation
            up[0] = 0;
            up[1] = 1;
            up[2] = 0;

            // NB: fall through
        }

        // locked straight down, but rotation around Z axis permitted.
        case "2D":
        {
            // look straight down
            eye[0] = lookat[0];
            eye[1] = lookat[1];
            break;
        }
    }

    return { eye: eye,
             lookat: lookat,
             up: up };
}

DefaultCameraControls.prototype.gotoELU = function(eye, lookat, up, dtime)
{
    if (!Number.isFinite(eye[0]))
        console.log("bad!");

    var elu = this.fixup(eye, lookat, up);
    eye = elu.eye; lookat = elu.lookat; up = elu.up;

    var time = new Date().getTime();

    // compute our current position and derivative so we can construct
    // a new polynomial.
    var eyeInterp = interpolate(this.eye0, this.eye0deriv, this.eye1, this.eluTime0, this.eluTime1, time);
    var lookatInterp = interpolate(this.lookat0, this.lookat0deriv, this.lookat1, this.eluTime0, this.eluTime1, time);
    var upInterp = interpolate(this.up0, this.up0deriv, this.up1, this.eluTime0, this.eluTime1, time);

    this.eye0  = eyeInterp.Y;
    this.eye0deriv = eyeInterp.Yderiv;
    this.eye1  = eye;

    this.lookat0  = lookatInterp.Y;
    this.lookat0deriv = lookatInterp.Yderiv;
    this.lookat1  = lookat;

    this.up0  = upInterp.Y;
    this.up0deriv = upInterp.Yderiv;
    this.up1  = up;

    this.eluTime0 = time;
    this.eluTime1 = time + dtime;

    this.layer.vc.requestContinuousRedraw(this.layer.name+":camera-elu");
}

DefaultCameraControls.prototype.setupLayer = function(layer, mtime, canvas_width, canvas_height)
{
    var time = new Date().getTime();

    // TODO: animate viewport changes
    layer.viewport = [ 0, 0, canvas_width, canvas_height ];

    // TODO: animate layer changes
    var layer_width = layer.viewport[2] - layer.viewport[0];
    var layer_height = layer.viewport[3] - layer.viewport[1];
    layer.P = gluPerspective(40, layer_width / layer_height, 0.1, 50000);

    if (true) {
        var eyeInterp = interpolate(this.eye0, this.eye0deriv, this.eye1, this.eluTime0, this.eluTime1, time);
        layer.eye = eyeInterp.Y;

        var lookatInterp = interpolate(this.lookat0, this.lookat0deriv, this.lookat1, this.eluTime0, this.eluTime1, time);
        layer.lookat = lookatInterp.Y;

        var upInterp = interpolate(this.up0, this.up0deriv, this.up1, this.eluTime0, this.eluTime1, time);
        layer.up = upInterp.Y;

        if (time >= this.eluTime1) {
            layer.vc.requestContinuousRedrawEnd(layer.name+":camera-elu");
        }
    }

    layer.V = elu_to_matrix(layer.eye, layer.lookat, layer.up);

    // animate background color changes
    if (true) {
        var interp = interpolate(this.background0, this.background0deriv, this.background1,
                                 this.backgroundTime0, this.backgroundTime1, time);

        layer.background = interp.Y;

        if (time >= this.backgroundTime1)
            layer.vc.requestContinuousRedrawEnd(layer.name+":camera-rgba");
    }
}

DefaultCameraControls.prototype.onEvent = function(layer, type, e)
{
    switch (type)
    {
        case "onmousewheel": {
            if ((this.cameraControlMask & 1) == 0)
                return false;

            // Firefox and Chrome handle "wheel" events differently. For example,
            // in Chrome, holding SHIFT swaps the scroll axis from Y to X. This
            // treats ALL scroll events as vertical scrolling for now, which
            // handles the problem. If we ever decide side scrolling is important
            // then this will obviously break.
            var delta = e.deltaY == 0 ? e.deltaX : e.deltaY;

            var speed = e.shiftKey ? 1.1 : 1.05;
            var ratio = (delta > 0) ? speed : 1.0/speed;
            var neweye = [ 0, 0, 0];

            for (var i = 0; i < 3; i++) {
                //                var diff = layer.eye[i] - layer.lookat[i];
                var diff = this.eye1[i] - this.lookat1[i];
                neweye[i] = diff * ratio + layer.lookat[i];
            }

            var mp = this.computeManipulationPoint(layer, e.xy[0], e.xy[1]);
            var mv = windowSpacePanTo(mp, e.xy[0], layer.viewport[3] - e.xy[1], false,
                                      layer.P, layer.viewport,
                                      neweye, layer.lookat, layer.up);

            this.gotoELU(vec_add(mv, neweye), vec_add(mv, layer.lookat), layer.up, this.animateMS);

            // NB: Don't consume the event. Some VX
            // applications want to eaves drop on these.
            return false;
        }

        case "onmousedown": {
            this.manipulationPoint = this.computeManipulationPoint(layer, e.xy[0], e.xy[1]);
            this.mouseDown = 1;

            this.lastRotateX = e.xy[0];
            this.lastRotateY = e.xy[1];
            return false;
        }

        case "onmousemove": {
            var eye = layer.eye, lookat = layer.lookat, up = layer.up;

            var pixelsToRadians = Math.PI / Math.max(layer.viewport[2], layer.viewport[3]);
            var manipulationPoint = this.manipulationPoint;

            if (this.mouseDown) {

                var dx = e.xy[0] - this.lastRotateX;
                var dy = e.xy[1] - this.lastRotateY;

                if (e.button==0 && !(e.buttons == 2) && !e.shiftKey && !e.ctrlKey && !e.altKey) {

                    // translate!
                    if ((this.cameraControlMask & 2) == 0)
                        return false;

                    var mv = windowSpacePanTo(manipulationPoint, e.xy[0], layer.viewport[3] - e.xy[1], false,
                                              layer.P, layer.viewport,
                                              this.eye1, this.lookat1, this.up1);

                    this.gotoELU(vec_add(mv, this.eye1), vec_add(mv, this.lookat1), this.up1, this.animateMS);
                    this.layer.vc.requestRedraw();

                    // NB: Don't consume the event. Some VX
                    // applications want to eaves drop on these.
                    return false;
                } else if (e.button==2 || e.buttons==2) { // if (this.camera.interfaceMode > 2.0) {

                    if ((this.cameraControlMask & 4) == 0)
                        return false;

                    var only_roll = e.shiftKey && e.ctrlKey;
                    var only_pitch = e.shiftKey && !e.ctrlKey;
                    var only_yaw = !e.shiftKey && e.ctrlKey;

                    var qcum = [ 1, 0, 0, 0 ];

                    var p2eye = vec_subtract(eye, lookat);
                    var left = vec_crossproduct(layer.up, p2eye);

                    if (only_roll) {
                        var cx = layer.viewport[0] + layer.viewport[2]/2;
                        var cy = layer.viewport[1] + layer.viewport[3]/2;

                        var theta0 = Math.atan2(this.lastRotateY - cy, this.lastRotateX - cx);
                        var theta1 = Math.atan2(e.xy[1] - cy, e.xy[0] - cx);

                        qcum = quat_multiply(qcum, angleaxis_to_quat(theta1 - theta0, p2eye));

                    } else if (only_yaw) {

                        qcum = quat_multiply(qcum, angleaxis_to_quat(-dx * pixelsToRadians, up));

                    } else if (only_pitch) {

                        qcum = quat_multiply(qcum, angleaxis_to_quat(-dy * pixelsToRadians, left));

                    } else {

                        qcum = quat_multiply(qcum, angleaxis_to_quat(-dx * pixelsToRadians, up));
                        qcum = quat_multiply(qcum, angleaxis_to_quat(-dy * pixelsToRadians, left));
                    }

                    this.lastRotateX = e.xy[0];
                    this.lastRotateY = e.xy[1];

                    var toEyeVec = vec_subtract(this.eye1, this.lookat1);
                    var newToEyeVec = quat_rotate(qcum, toEyeVec);
                    var neweye = vec_add(this.lookat1, newToEyeVec);
                    var newup = quat_rotate(qcum, this.up1);

                    //                    var newlookat = this.lookat1;
                    var newlookat = this.computeManipulationPoint(this.layer, this.layer.viewport[2] / 2, this.layer.viewport[3] / 2);
                    // drop homogeneous coordinates
                    newlookat = [ newlookat[0], newlookat[1], newlookat[2] ];

                    this.gotoELU(neweye, newlookat, newup, this.animateMS);
                    this.layer.vc.requestRedraw();

                    // NB: Don't consume the event. Some VX
                    // applications want to eaves drop on these.
                    return false;
                }
                return false; // ??? XXX
            }

            return false;
        }

        case "onmouseup": {
            console.log("vx_camera onmouseup");
            this.mouseDown = false;
            break;
        }

        case "ontouchstart": {
            if (e.touches.length == 1) {
                this.manipulationPoint = this.computeManipulationPoint(layer, e.xy[0], e.xy[1]);
            }

            if (e.touches.length == 2) {
                this.manipulationPoint = null;
                this.lastTouchCenter = null;
            }

            return false;
        }

        case "ontouchmove": {
            var pixelsToRadians = Math.PI / Math.max(layer.viewport[2], layer.viewport[3]);

            if (e.touches.length == 1) {
                // Pan
                if ((this.cameraControlMask & 2) == 2) {
                    var mv = windowSpacePanTo(this.manipulationPoint, e.xy[0], layer.viewport[3] - e.xy[1], false,
                                              layer.P, layer.viewport,
                                              this.eye1, this.lookat1, this.up1);

                    this.gotoELU(vec_add(mv, this.eye1), vec_add(mv, this.lookat1), this.up1, this.touchAnimateMS);
                    this.layer.vc.requestRedraw();
                }
            }

            if (e.touches.length == 2) {
                var eye = layer.eye, lookat = layer.lookat, up = layer.up;
                var t0 = e.touches[0];
                var t1 = e.touches[1];
                var tdx = t0.clientX - t1.clientX;
                var tdy = t0.clientY - t1.clientY;

                var dist = Math.sqrt(tdx*tdx + tdy*tdy);
                var center = [ (t0.clientX + t1.clientX)/2, (t0.clientY + t1.clientY)/2 ];

                if (this.lastTouchCenter != null) {
                    if (this.cameraControlMask & 5 == 0)
                        return false;

                    var dx = center[0] - this.lastTouchCenter[0];
                    var dy = center[1] - this.lastTouchCenter[1];

                    var qcum = [ 1, 0, 0, 0 ];

                    var p2eye = vec_subtract(eye, lookat);
                    var left = vec_crossproduct(up, p2eye);

                    qcum = quat_multiply(qcum, angleaxis_to_quat(-dx * pixelsToRadians, up));
                    qcum = quat_multiply(qcum, angleaxis_to_quat(-dy * pixelsToRadians, left));

                    var toEyeVec = vec_subtract(this.eye1, this.lookat1);
                    var newlookat = this.lookat1;
                    var newToEyeVec = toEyeVec;
                    var newup = this.up1;

                    // Rotate
                    if ((this.cameraControlMask & 4) == 4) {
                        newToEyeVec = quat_rotate(qcum, toEyeVec);
                        newup = quat_rotate(qcum, this.up1);
                    }

                    var neweye = vec_add(this.lookat1, newToEyeVec);

                    // Zoom
                    if ((this.cameraControlMask & 1) == 1) {
                        var ratio = this.lastTouchDist / dist;
                        for (var i = 0; i < 3; i++) {
                            var diff = neweye[i] - newlookat[i];
                            neweye[i] = diff * ratio + newlookat[i];
                        }
                    }

                    this.gotoELU(neweye, newlookat, newup, this.touchAnimateMS);
                    this.layer.vc.requestRedraw();
                }

                this.lastTouchCenter = center;
                this.lastTouchDist = dist;
            }


            break;
        }

        case "ontouchend": {
            if (e.touches.length == 0) {
                console.log("vx_camera ontouchend");
                this.touchDown = false;
                this.manipulationPoint = null;
            }
            break;
        }

        case "onmouseclick": {
            break;
        }
    }

    return false;
}
