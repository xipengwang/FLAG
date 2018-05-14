"use strict";

var uiAnimateMs = 50; // 850 is nice for sloppy. 50 for "live"

// TODO: Maybe don't use bubble sort.
function stable_sort(arr, fn)
{
    var changed = 1;

    while (changed) {
        changed = 0;

        for (var i = 0; i+1 < arr.length; i++) {
            if (fn(arr[i], arr[i+1]) > 0) {
                var tmp = arr[i];
                arr[i] = arr[i+1];
                arr[i+1] = tmp;
                changed = 1;
            }
        }
    }
}

function clamp(v, min, max)
{
    if (v < min)
        return min;
    if (v > max)
        return max;
    return v;
}

function mod2pi_ref(ref, v)
{
    while (v > ref + Math.PI)
        v -= 2*Math.PI;
    while (v < ref - Math.PI)
        v += 2*Math.PI;
    return v;
}

function mod2pi(v)
{
    return mod2pi_ref(0, v);
}

function make_shader(gl, type, src)
{
    var shader = gl.createShader(type);
    gl.shaderSource(shader, src);
    gl.compileShader(shader);

    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
        alert("make_shader error: "+gl.getShaderInfoLog(shader));
        return null;
    }

    return shader;
}

function throwOnGLError(err, funcName, args) {
  throw WebGLDebugUtils.glEnumToString(err) + " was caused by call to: " + funcName;
};

function logGLCall(functionName, args) {
   console.log("gl." + functionName + "(" +
      WebGLDebugUtils.glFunctionArgsToString(functionName, args) + ")");
}

function VwCanvas(canvasid)
{
    this.canvasid = canvasid;
    this.glcanvas = document.getElementById(canvasid);
    this.gl = this.glcanvas.getContext("webgl", {preserveDrawingBuffer: true });
    if (this.gl == null)
        this.gl = this.glcanvas.getContext("experimental-webgl");

    if (this.gl == null)
        this.gl = this.glcanvas.getContext("experimental-webgl");

    this.on_draw_callbacks = [];
    this.on_layer_callbacks = [];

    this.glcanvas.addEventListener('contextmenu', function(e) { e.preventDefault(); });
    this.glcanvas.oncontextmenu = function() { return false; };

    this.glcanvas.focus();

//    this.gl = WebGLDebugUtils.makeDebugContext(this.gl, throwOnGLError, logGLCall);

    var gl = this.gl;
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LESS);

    gl.enable(gl.BLEND);
    gl.enable(gl.SCISSOR_TEST);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
//    gl.enable(gl.CULL_FACE);
//    gl.cullFace(gl.BACK);

    this.layers = [];

    var _vw = this;

    this.glcanvas.onmousewheel = function(e) { _vw.onmousewheel(e); };
    this.glcanvas.addEventListener('DOMMouseScroll',
                                   function(e) {
                                       e.wheelDelta = Math.max(-1, Math.min(1, (e.wheelDelta || -e.detail))); //FF hack
                                       _vw.onmousewheel(e);
                                   });
    this.glcanvas.onmousedown  = function(e) { _vw.onmousedown(e); };
    this.glcanvas.onmouseup    = function(e) { _vw.onmouseup(e); };
    this.glcanvas.onmousemove  = function(e) { _vw.onmousemove(e); };
    this.glcanvas.onclick      = function(e) { _vw.onmouseclick(e); };

    this.glcanvas.ontouchstart = function(e) { _vw.ontouchstart(e); };
    this.glcanvas.ontouchmove  = function(e) { _vw.ontouchmove(e); };
    this.glcanvas.ontouchend   = function(e) { _vw.ontouchend(e); };

    this.glcanvas.onkeypress   = function(e) { _vw.onkeypress(e); };
    this.glcanvas.onkeydown    = function(e) { _vw.onkeydown(e); };
    this.glcanvas.onkeyup      = function(e) { _vw.onkeyup(e); };

     this.glcanvas.force_redraw = false;

    var oldfn = document.onmouseup;
    document.onmouseup = function(e) { _vw.onmouseup(e); if (oldfn) return oldfn(e); };

    window.setInterval(function() {
        if (_vw.dirty) {
            _vw.dirty = 0;

           if (_vw.glcanvas.force_redraw)
              _vw.draw();
           else
              requestAnimationFrame(function() { _vw.draw(); });
        }
    }, 30);
}

// compute the coordinates in a way that's consistent with OpenGL
VwCanvas.prototype.computeEventCoordinates = function(clientX, clientY)
{
    var devicePixelRatio = window.devicePixelRatio || 1;

    var xy = [ (clientX - this.glcanvas.offsetLeft) * devicePixelRatio,
               (clientY - this.glcanvas.offsetTop) * devicePixelRatio ];
    return xy;
}

// the first time a finger goes down, we elect a layer
VwCanvas.prototype.ontouchstart = function(e)
{
    e.xys = [];
    e.xys[0] = this.computeEventCoordinates(e.changedTouches[0].clientX, e.changedTouches[0].clientY);
    e.preventDefault();

    var ch = this.glcanvas.height;

    if (this.touchDownLayer == null) {

        // process from "topmost" to "bottommost"
        for (var i = this.layers.length - 1; i >= 0; i--) {
            var layer = this.layers[i];
            var viewport = layer.renderinfo.viewport;
            var touch = e.touches[0];

            var xy = e.xys[0];
            if (xy[0] >= viewport[0] && xy[0] < viewport[0] + viewport[2] &&
                (ch - xy[1]) >= viewport[1] && (ch - xy[1]) < viewport[1] + viewport[3]) {

                this.touchDownLayer = layer;
            }
        }

        return true;
    } else {
        return this.touchDownLayer.ontouchstart(e);
    }

    return false;
};

VwCanvas.prototype.onkeydown = function(e)
{
    // 9: Tab, 8: backspace
    if (e.keyCode == 9 || e.keyCode == 8)
        e.preventDefault();

    var layer = null;
    if (layer == null)
        layer = this.touchDownLayer;
    if (layer == null)
        layer = this.mouseDownLayer;
    if (layer == null)
        layer = this.layers[0];

    if (layer)
        layer.onkeydown(e);
};

VwCanvas.prototype.onkeyup = function(e)
{
    var layer = null;
    if (layer == null)
        layer = this.touchDownLayer;
    if (layer == null)
        layer = this.mouseDownLayer;
    if (layer == null)
        layer = this.layers[0];

    if (layer)
        layer.onkeyup(e);
};

VwCanvas.prototype.onkeypress = function(e)
{
    e.xy = this.computeEventCoordinates(e.clientX, e.clientY);
    e.preventDefault();

    var layer = null;
    if (layer == null)
        layer = this.touchDownLayer;
    if (layer == null)
        layer = this.mouseDownLayer;
    if (layer == null)
        layer = this.layers[0];

    if (layer)
        layer.onkeypress(e);
};

VwCanvas.prototype.ontouchmove = function(e)
{
    // This is a workaround.
    // It seems that changedTouches contains ALL touches in ontouchmove instead of only those that changed.
    // For now, send them all.
    // TODO: only send touches that have actually changed.
    e.xys = [];
    for (var i = 0; i < e.changedTouches.length; i++) {
	    e.xys[i] = this.computeEventCoordinates(e.changedTouches[i].clientX, e.changedTouches[i].clientY);
    }
    e.preventDefault();

    if (this.touchDownLayer) {
        return this.touchDownLayer.ontouchmove(e);
    }
};

VwCanvas.prototype.ontouchend = function(e)
{
    e.xys = [];
    for (var i = 0; i < e.changedTouches.length; i++) {
	    e.xys[i] = this.computeEventCoordinates(e.changedTouches[i].clientX, e.changedTouches[i].clientY);
    }
    e.preventDefault();

    if (this.touchDownLayer) {
        return this.touchDownLayer.ontouchend(e);
    }

    return false;
};

VwCanvas.prototype.onmousewheel = function(e)
{
    e.xy = this.computeEventCoordinates(e.clientX, e.clientY);
    var ch = this.glcanvas.height;

    // process from "topmost" to "bottommost"
    for (var i = this.layers.length - 1; i >= 0; i--) {
        var layer = this.layers[i];
        var viewport = layer.renderinfo.viewport;

        if (e.xy[0] >= viewport[0] && e.xy[0] < viewport[0] + viewport[2] &&
            (ch - e.xy[1]) >= viewport[1] && (ch - e.xy[1]) < viewport[1] + viewport[3]) {
            if (this.layers[i].onmousewheel(e))
                return true;
        }
    }

    return false;
};

VwCanvas.prototype.onmousedown = function(e)
{
    e.xy = this.computeEventCoordinates(e.clientX, e.clientY);
    var ch = this.glcanvas.height;

    // process from "topmost" to "bottommost"
    for (var i = this.layers.length - 1; i >= 0; i--) {

        var layer = this.layers[i];
        var viewport = layer.renderinfo.viewport;

        if (e.xy[0] >= viewport[0] && e.xy[0] < viewport[0] + viewport[2] &&
            (ch - e.xy[1]) >= viewport[1] && (ch - e.xy[1]) < viewport[1] + viewport[3]) {
            this.mouseDownLayer = layer;
            return layer.onmousedown(e);
            // XXX should return onmousedown?
//            return true;
        }
    }

    return false;
};

VwCanvas.prototype.onmouseclick = function(e)
{
    e.xy = this.computeEventCoordinates(e.clientX, e.clientY);
    var ch = this.glcanvas.height;

    // process from "topmost" to "bottommost"
    for (var i = this.layers.length - 1; i >= 0; i--) {

        var layer = this.layers[i];
        var viewport = layer.renderinfo.viewport;

        if (e.xy[0] >= viewport[0] && e.xy[0] < viewport[0] + viewport[2] &&
            (ch - e.xy[1]) >= viewport[1] && (ch - e.xy[1]) < viewport[1] + viewport[3]) {
            this.mouseDownLayer = layer;
            return layer.onmouseclick(e);
        }
    }

    return false;
};

VwCanvas.prototype.onmouseleave = function(e)
{
    e.xy = this.computeEventCoordinates(e.clientX, e.clientY);

    return this.onmouseup(e);
}

VwCanvas.prototype.onmouseup = function(e)
{
    e.xy = this.computeEventCoordinates(e.clientX, e.clientY);

    if (this.mouseDownLayer) {
        var ret = this.mouseDownLayer.onmouseup(e);
        this.mouseDownLayer = null;
        return ret;
    }
    return false;
};

VwCanvas.prototype.onmousemove = function(e)
{
    e.xy = this.computeEventCoordinates(e.clientX, e.clientY);
   var ch = this.glcanvas.height;

    // if they drag the mouse out of the window, release the mouse button, then
    // re-enter the window, automatically generate the "lost" mouseup event.
    // NB: Broken because e.button==0 means left button. (No way to tell if buttons released?)
//    if (e.button == 0 && this.mouseDownLayer != null) {
//        return this.onmouseup(e);
//    }

    if (this.mouseDownLayer)
        return this.mouseDownLayer.onmousemove(e);


   // process from "topmost" to "bottommost"
    for (var i = this.layers.length - 1; i >= 0; i--) {

        var layer = this.layers[i];
        var viewport = layer.renderinfo.viewport;

        if (e.xy[0] >= viewport[0] && e.xy[0] < viewport[0] + viewport[2] &&
            (ch - e.xy[1]) >= viewport[1] && (ch - e.xy[1]) < viewport[1] + viewport[3]) {
            return layer.onmousemove(e);
        }
    }

    return false;
};

VwCanvas.prototype.getWidth = function()
{
    return this.glcanvas.width;
};

VwCanvas.prototype.getHeight = function()
{
    return this.glcanvas.height;
};


VwCanvas.prototype.getLayer = function(name)
{
    return this.getLayerNotify(name, null);
}

// calls the callback if the layer is new
VwCanvas.prototype.getLayerNotify = function(name, callback)
{
    for (var i = 0; i < this.layers.length; i++) {
        vl = this.layers[i];
        if (vl.name == name)
            return vl;
    }

    var vl = new VwLayer(this);
    vl.name = name;
    this.layers.push(vl);

    vl.on_buffer_callbacks = [];

    for (var i = 0; i < this.on_layer_callbacks.length; i++) {
        this.on_layer_callbacks[i](vl);
    }

    if (callback)
        callback(vl);

    return vl;
}

VwCanvas.prototype.draw = function()
{
    stable_sort(this.layers, function(a, b) { return a.draworder - b.draworder });
//    this.layers.sort(function(a, b) { return a.draworder - b.draworder });

    var gl = this.gl;

    for (var i = 0; i < this.layers.length; i++) {
        this.layers[i].draw();
    }

    gl.finish();

    for (var i = 0; i < this.on_draw_callbacks.length; i++)
        this.on_draw_callbacks[i](this);
}

VwCanvas.prototype.onDirty = function()
{
    // will be polled by timer
    this.dirty = 1;
}

function EnableStack(initvalue)
{
    this.stack = [ ];
    this.stack.push(initvalue);
}

EnableStack.prototype.get = function()
{
    return this.stack[this.stack.length - 1];
}

EnableStack.prototype.push = function()
{
    this.stack.push(this.get());
}

EnableStack.prototype.set = function(v)
{
    this.stack[this.stack.length-1] = v;
}

EnableStack.prototype.pop = function()
{
    this.stack.pop();
}

function MatrixStack(Minit)
{
    this.stack = [];
    this.stack.push(mat_copy(Minit));
}

MatrixStack.prototype.get = function()
{
    return this.stack[this.stack.length - 1];
}

MatrixStack.prototype.push = function()
{
    this.stack.push(mat_copy(this.get()));
}

MatrixStack.prototype.pop = function()
{
    this.stack.pop();
}

MatrixStack.prototype.multiply = function(M)
{
    this.stack[this.stack.length-1] = mat_multiply(this.stack[this.stack.length-1], M);
}

MatrixStack.prototype.set = function(M)
{
    this.stack[this.stack.length-1] = mat_copy(M);
}

/////////////////////////////////////////////////////////
// projection, view, model... all 16 len vectors
function VwStack(P, V, M)
{
    this.M = new MatrixStack(M);
    this.P = new MatrixStack(P);
    this.V = new MatrixStack(V);
    this.depth_test = new EnableStack(1);
}


/////////////////////////////////////////////////////////
function InterpolatingLayerViewportComputer(viewport0, mtime0,
                                            viewport1, mtime1)
{
    this.viewport0 = viewport0;
    this.mtime0 = mtime0;

    this.viewport1 = viewport1;
    this.mtime1 = mtime1;
}

InterpolatingLayerViewportComputer.prototype.scaleviewport = function(vp, canvas_width, canvas_height)
{
    var r = [ 0, 0, 0, 0 ];
    var s = [ canvas_width, canvas_height, canvas_width, canvas_height ];

    for (var i = 0; i < 4; i++) {
        r[i] = vp[i];

        if (vp[i] >= 0 && vp[i] <= 1)
            r[i] = vp[i]*s[i];
        else if (vp[i] >= -1 && vp[i] < 0)
            r[i] = s[i] + vp[i]*s[i];
        else if (vp[i] < 0)
            r[i] = s[i] + vp[i];
    }

    return r;
}

InterpolatingLayerViewportComputer.prototype.compute = function(canvas_width, canvas_height, mtime)
{
    var alpha1 = (this.mtime1 == this.mtime0) ? 1 : clamp((mtime - this.mtime0) / (this.mtime1 - this.mtime0), 0, 1);
    var alpha0 = 1.0 - alpha1;

    var vp0 = this.scaleviewport(this.viewport0, canvas_width, canvas_height);
    var vp1 = this.scaleviewport(this.viewport1, canvas_width, canvas_height);

    var v = vec_add(vec_scale(vp0, alpha0), vec_scale(vp1, alpha1));
    v.done = (mtime >= this.mtime1);

//    v = [ 0, 0, canvas_width, canvas_height ];
//    console.log(v);

    return v;
}

function make_elu(eye, lookat, up)
{
    var elu = new Object();
    elu.eye = eye;
    elu.lookat = lookat;
    elu.up = up;
    return elu;
}

function fixup_elu(elu)
{
    if (true) {
        // 2.5D mode
        var original_dist = vec_distance(elu.eye, elu.lookat);

        // always looking at z=0 (XXX use manip point?)
        elu.lookat[2] = 0;

        var lookdir = vec_normalize(vec_subtract(elu.lookat, elu.eye));
        // make sure horizon is level
        var left = vec_crossproduct(elu.up, lookdir);
        left[2] = 0;
        left = vec_normalize(left);
        elu.up = vec_normalize(vec_crossproduct(lookdir, left));

        var dir = vec_crossproduct(elu.up, left);

        elu.eye = vec_add(elu.lookat, vec_scale(dir, original_dist));
    }

    return elu;
}

/////////////////////////////////////////////////////////
function InterpolatingViewComputer(elu0, mtime0,
                                   elu1, mtime1,
                                   fixed_look_distance)
{
    this.eye0 = elu0.eye;
    this.lookat0 = elu0.lookat;
    this.up0 = elu0.up;
    this.mtime0 = mtime0;

    this.eye1 = elu1.eye;
    this.lookat1 = elu1.lookat;
    this.up1 = elu1.up;
    this.mtime1 = mtime1;

    this.fixed_look_distance = fixed_look_distance;
}

InterpolatingViewComputer.prototype.compute = function(mtime)
{
    var alpha1 = (this.mtime1 == this.mtime0) ? 1 : clamp((mtime - this.mtime0) / (this.mtime1 - this.mtime0), 0, 1);

    // this interpolation function is designed to smooth the end of an
    // animation and is based on a linearly decreasing slope that
    // reaches zero at alpha1 = 1;
    alpha1 = 2*alpha1 - alpha1*alpha1;

    var alpha0 = 1.0 - alpha1;

    if (!isFinite(alpha1))
        console.log("non finite alpha1");

    var eye    = vec_add(vec_scale(this.eye0,    alpha0), vec_scale(this.eye1,    alpha1));
    var lookat = vec_add(vec_scale(this.lookat0, alpha0), vec_scale(this.lookat1, alpha1));
    var up     = vec_add(vec_scale(this.up0,     alpha0), vec_scale(this.up1,     alpha1));

    // constrain distance between eye and lookat to be the same distance it was before.
    if (this.fixed_look_distance) {
        var lookdist = vec_distance(this.eye0, this.lookat0);
        var lookdir = vec_normalize(vec_subtract(eye, lookat));
        eye = vec_add(lookat, vec_scale(lookdir, lookdist));
    }

    var elu = new Object();
    elu.eye = eye;
    elu.lookat = lookat;
    elu.up = up;
    elu.V = lookat_to_matrix(eye, lookat, up);
    elu.done = (mtime >= this.mtime1);
    return elu;
}

/////////////////////////////////////////////////////////
function InterpolatingProjectionComputer(perspectiveness0, fovy_degrees0, zclip_near0, zclip_far0, mtime0,
                                         perspectiveness1, fovy_degrees1, zclip_near1, zclip_far1, mtime1,
                                         layer)
{
    this.perspectiveness0 = perspectiveness0;
    this.fovy_degrees0 = fovy_degrees0;
    this.zclip_near0 = zclip_near0;
    this.zclip_far0 = zclip_far0;
    this.mtime0 = mtime0;

    this.perspectiveness1 = perspectiveness1;
    this.fovy_degrees1 = fovy_degrees1;
    this.zclip_near1 = zclip_near1;
    this.zclip_far1 = zclip_far1;
    this.mtime1 = mtime1;

    this.layer = layer;
}

InterpolatingProjectionComputer.prototype.compute = function(layerwidth, layerheight, mtime)
{
    var elu = this.layer.viewComputer.compute(mtime);

    var alpha1 = (this.mtime1 == this.mtime0) ? 1 : clamp((mtime - this.mtime0) / (this.mtime1 - this.mtime0), 0, 1);
    var alpha0 = 1.0 - alpha1;

    var aspect = layerwidth / layerheight;

    var fovy_degrees     = alpha0 * this.fovy_degrees0     + alpha1 * this.fovy_degrees1;
    var perspectiveness  = alpha0 * this.perspectiveness0  + alpha1 * this.perspectiveness1;
    var zclip_near       = alpha0 * this.zclip_near0       + alpha1 * this.zclip_near1;
    var zclip_far        = alpha0 * this.zclip_far0        + alpha1 * this.zclip_far1;

    var PP = gluPerspective(fovy_degrees, aspect, zclip_near, zclip_far);

    var lookdist = vec_distance(elu.eye, elu.lookat);
    var PO = glOrtho(-lookdist * aspect/2, lookdist*aspect / 2, -lookdist/2, lookdist/2,
                     -zclip_far, zclip_far);
    var perspectiveness_scaled = Math.pow(perspectiveness, 3);
    var P = mat_identity(4);

    for (var i = 0; i < 16; i++)
        P[i] = perspectiveness_scaled*PP[i] + (1-perspectiveness_scaled)*PO[i];

    var projinfo = new Object();
    projinfo.fovy_degrees = fovy_degrees;
    projinfo.zclip_near = zclip_near;
    projinfo.zclip_far = zclip_far;
    projinfo.P = P;
    projinfo.done = (mtime >= this.mtime1);
    projinfo.perspectiveness = perspectiveness;
    return projinfo;
}
/////////////////////////////////////////////////////////
function DefaultCameraControls(layer)
{
    this.layer = layer;
    this.touchManipulationPoints = new Object();
    this.lastRotateX = 0;
    this.lastRotateY = 0;
    this.touchControlsEnabled = true;
    this.interfaceMode = "3D";
    this.cameraControlMask = 7; // ZOOM(1) + PAN(2) + ROTATE(4)
}

DefaultCameraControls.prototype.onkeydown = function(e) {
    return false;
};

DefaultCameraControls.prototype.onkeyup = function(e) {
    return false;
};

DefaultCameraControls.prototype.onkeypress = function(e) {
    return false;
};

DefaultCameraControls.prototype.ontouchstart = function(e) {

    // always recompute manipulation point so that if touch controls
    // are enabled later that we have an MP. (?)
    if (!this.touchControlsEnabled)
        return false;

/*    for (var i = 0; i < e.changedTouches.length; i++) {
        var touch = e.changedTouches[i];
        var mp = this.layer.computeManipulationPoint(e.xys[i][0], e.xys[i][1]);
        this.touchManipulationPoints[touch.identifier] = mp;
    }
*/

    if (e.touches.length == 2)
        this.lastTouchCenter = null;

    if (e.touches.length > 2)
        return false;

    return true;
};

DefaultCameraControls.prototype.ontouchmove = function(e) {

    var handled = false;

    if (!this.touchControlsEnabled)
        return false;

    var renderinfo = this.layer.renderinfo;
    var eye = renderinfo.eye, lookat = renderinfo.lookat, up = renderinfo.up;
    var pixelsToRadians = Math.PI / Math.max(renderinfo.viewport[2], renderinfo.viewport[3]);

    if (e.touches.length == 1) {
        var touch = e.touches[0];
        var mp = null;
        if (!this.touchManipulationPoints.hasOwnProperty("MP"+touch.identifier))
            this.touchManipulationPoints["MP"+touch.identifier] = this.layer.computeManipulationPoint(e.xys[0][0], e.xys[0][1]);

        var mp = this.touchManipulationPoints["MP"+touch.identifier];
        var xy = e.xys[0];
        var mv = windowSpacePanTo(mp, xy[0], renderinfo.viewport[3] - xy[1], false,
                                  renderinfo.projectionMatrix, renderinfo.viewport,
                                  eye, lookat, up);

        this.layer.viewComputer = new InterpolatingViewComputer(make_elu(eye, lookat, up),
                                                                renderinfo.mtime,
                                                                fixup_elu(make_elu(vec_add(mv, eye), vec_add(mv, lookat), up)),
                                                                new Date().getTime() + uiAnimateMs,
                                                                false);
        console.log("ontouchmove");
        this.layer.canvas.onDirty();

        handled = true;
    }

    if (e.touches.length == 2) {
        var t0 = e.touches[0];
        var t1 = e.touches[1];
        var dist = Math.sqrt((t0.clientX - t1.clientX)*(t0.clientX - t1.clientX) + (t0.clientY - t1.clientY)*(t0.clientY - t1.clientY));
        var center = [ (t0.clientX + t1.clientX) / 2, (t0.clientY + t1.clientY) / 2 ];

        if (this.lastTouchCenter != null) {
            var dx = center[0] - this.lastTouchCenter[0];
            var dy = center[1] - this.lastTouchCenter[1];

            var qcum = [ 1, 0, 0, 0 ];

            var p2eye = vec_subtract(eye, lookat);
            var left = vec_crossproduct(renderinfo.up, p2eye);

            qcum = quat_multiply(qcum, angleaxis_to_quat(-dx * pixelsToRadians, up));
            qcum = quat_multiply(qcum, angleaxis_to_quat(-dy * pixelsToRadians, left));

            // XXX this assumes that the view computer defines these fields...
            var eye1 = this.layer.viewComputer.eye1, lookat1 = this.layer.viewComputer.lookat1, up1 = this.layer.viewComputer.up1;

            var toEyeVec = vec_subtract(eye1, lookat1);
            var newToEyeVec = quat_rotate(qcum, toEyeVec);
            var neweye = vec_add(lookat1, newToEyeVec);
            var newup = quat_rotate(qcum, up1);
            var newlookat = lookat1;

            var ratio = this.lastTouchDistance / dist;

            for (var i = 0; i < 3; i++) {
                var diff = neweye[i] - newlookat[i];
                neweye[i] = diff * ratio + newlookat[i];
            }

/*
  // zoom works pretty well without the added potential complications of picking a manipulation point.
            var xy = this.computeEventCoordinates((t0.clientX + t1.clientX) / 2.0, (t0.clientY + t1.clientY) / 2.0);

            var mv = windowSpacePanTo(mp, xy[0], renderinfo.viewport[3] - xy[1], false,
                                      renderinfo.projectionMatrix, renderinfo.viewport,
                                      neweye, newlookat, newup);
*/
            var mv = [ 0, 0, 0 ];
            this.layer.viewComputer = new InterpolatingViewComputer(make_elu(eye, lookat, up),
                                                                    renderinfo.mtime,
                                                                    fixup_elu(make_elu(vec_add(mv, neweye), vec_add(mv, newlookat), newup)),
                                                                    new Date().getTime() + uiAnimateMs,
                                                                    false);
            this.layer.canvas.onDirty();
        }

        this.lastTouchCenter = center;
        this.lastTouchDistance = dist;

        handled = true;

    } else {
        this.lastTouchCenter = null;
    }

    return handled;
};

DefaultCameraControls.prototype.ontouchend = function(e) {

    if (!this.touchControlsEnabled)
        return false;

    // If a finger has been lifted and two touches remain, a third finger was involved.
    if(e.touches.length > 2)
        return false;

    // a finger has been lifted. recompute all manipulation
    // points. (Imagine they were pinch zooming, release a finger. We
    // want to translate with the remaining finger based on where it
    // is now.)
    this.touchManipulationPoints = [ ];

    return true;
};

DefaultCameraControls.prototype.onmousewheel = function(e) {

    if ((this.cameraControlMask & 1) == 0)
        return false;

    var renderinfo = this.layer.renderinfo;
    var speed = e.shiftKey ? 1.5 : 1.1;
    var ratio = (e.wheelDelta > 0) ? speed : 1.0/speed;
    var eye = renderinfo.eye, lookat = renderinfo.lookat, up = renderinfo.up, viewport = renderinfo.viewport;
    var neweye = [ 0, 0, 0];

    for (var i = 0; i < 3; i++) {
        var diff = eye[i] - lookat[i];
        neweye[i] = diff * ratio + lookat[i];
    }

    var mp = this.layer.computeManipulationPoint(e.xy[0], e.xy[1]);
    var mv = windowSpacePanTo(mp, e.xy[0], renderinfo.viewport[3] - e.xy[1], false,
                              renderinfo.projectionMatrix, viewport,
                              neweye, lookat, up);


    this.layer.viewComputer = new InterpolatingViewComputer(make_elu(eye, lookat, up),
                                                            renderinfo.mtime,
                                                            fixup_elu(make_elu(vec_add(mv, neweye), vec_add(mv, lookat), up)),
                                                            new Date().getTime() + uiAnimateMs,
                                                            false);
    this.layer.canvas.onDirty();
    return false;
}

DefaultCameraControls.prototype.onmousedown = function(e) {

    this.manipulationPoint = this.layer.computeManipulationPoint(e.xy[0], e.xy[1]);
    this.mouseDown = 1;

    this.lastRotateX = e.xy[0];
    this.lastRotateY = e.xy[1];

    return false;
}

DefaultCameraControls.prototype.onmouseclick = function(e) {
    return false;
};

DefaultCameraControls.prototype.onmouseup = function(e) {
    this.mouseDown = 0;
    return false
};

DefaultCameraControls.prototype.onmousemoved = function(e) {
    var renderinfo = this.layer.renderinfo;
    var eye = renderinfo.eye, lookat = renderinfo.lookat, up = renderinfo.up;

    var pixelsToRadians = Math.PI / Math.max(renderinfo.viewport[2], renderinfo.viewport[3]);
    var manipulationPoint = this.manipulationPoint;

    if (this.mouseDown) {

        var dx = e.xy[0] - this.lastRotateX;
        var dy = e.xy[1] - this.lastRotateY;

        if (e.button==0 && !(e.buttons == 2) && !e.shiftKey && !e.ctrlKey && !e.altKey) {

            // translate!
            if ((this.cameraControlMask & 2) == 0)
                return false;

            var mv = windowSpacePanTo(manipulationPoint, e.xy[0], renderinfo.viewport[3] - e.xy[1], false,
                                      renderinfo.projectionMatrix, renderinfo.viewport,
                                      eye, lookat, up);

            this.layer.viewComputer = new InterpolatingViewComputer(make_elu(eye, lookat, up),
                                                                    renderinfo.mtime,
                                                                    fixup_elu(make_elu(vec_add(mv, eye), vec_add(mv, lookat), up)),
                                                                    new Date().getTime() + uiAnimateMs,
                                                              false);
            this.layer.canvas.onDirty();
        }
        else if (e.button==2 || e.buttons==2){ // if (this.camera.interfaceMode > 2.0) {

            if ((this.cameraControlMask & 4) == 0)
                return false;

            var only_roll = e.shiftKey && e.ctrlKey;
            var only_pitch = e.shiftKey && !e.ctrlKey;
            var only_yaw = !e.shiftKey && e.ctrlKey;

            var qcum = [ 1, 0, 0, 0 ];

            var p2eye = vec_subtract(eye, lookat);
            var left = vec_crossproduct(renderinfo.up, p2eye);

            if (only_roll) {
                var cx = renderinfo.viewport[0] + renderinfo.viewport[2]/2;
                var cy = renderinfo.viewport[1] + renderinfo.viewport[3]/2;

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

            // XXX this assumes that the view computer defines these fields...
            var eye1 = this.layer.viewComputer.eye1, lookat1 = this.layer.viewComputer.lookat1, up1 = this.layer.viewComputer.up1;

            var toEyeVec = vec_subtract(eye1, lookat1);
            var newToEyeVec = quat_rotate(qcum, toEyeVec);
            var neweye = vec_add(lookat1, newToEyeVec);
            var newup = quat_rotate(qcum, up1);
            var newlookat = lookat1;

/*            if (true) {
                // 2.5D mode

                // always looking at z=0 (XXX use manip point?)
                newlookat[2] = 0;

                var lookdir = vec_normalize(vec_subtract(newlookat, neweye));
                // make sure horizon is level
                var left = vec_crossproduct(newup, lookdir);
                left[2] = 0;
                left = vec_normalize(left);

                newup = vec_crossproduct(lookdir, left);
            }
*/
            this.layer.viewComputer = new InterpolatingViewComputer(make_elu(eye, lookat, up),
                                                                    renderinfo.mtime,
                                                                    fixup_elu(make_elu(neweye, newlookat, newup)),
                                                                    new Date().getTime() + uiAnimateMs,
                                                                    true);
            this.layer.canvas.onDirty();
        }
    }

    // NOTE: This may generate a lot of event messages.
    // For now it seems fine, and is needed by certain applications. i.e. joypad drive (mouse version)
    return false;
}

/////////////////////////////////////////////////////////
function VwLayer(canvas)
{
    this.canvas = canvas;

    this.default_eye = [ 0, 0, 10 ];
    this.default_lookat = [ 0, 0, 0 ];
    this.default_up = [ 0, 1, 0 ];

    this.default_perspectiveness = 1.0;
    this.default_fovy_degrees = 50;
    this.default_zclip_near = 0.1;
    this.default_zclip_far = 50000;

    this.draworder = 0;
    this.buffers = [];

    this.visible = true;

    var mtime = new Date().getTime();

    this.layerViewportComputer = new InterpolatingLayerViewportComputer([ 0, 0, 1, 1], mtime,
                                                                        [ 0, 0, 1, 1], mtime);

    this.projectionComputer = new InterpolatingProjectionComputer(this.default_perspectiveness, this.default_fovy_degrees,
                                                                  this.default_zclip_near, this.default_zclip_far, mtime,
                                                                  this.default_perspectiveness, this.default_fovy_degrees,
                                                                  this.default_zclip_near, this.default_zclip_far, mtime,
                                                                  this);

    this.viewComputer = new InterpolatingViewComputer(make_elu(this.default_eye, this.default_lookat, this.default_up),
                                                      mtime,
                                                      make_elu(this.default_eye, this.default_lookat, this.default_up),
                                                      mtime,
                                                      false);

    this.background_rgba = [ .20, .20, .20, 1];

    this.computeManipulationPoint = makeComputeManipulationPointXYPlane(0);

    this.renderinfo = this.makeRenderInfo();

    this.event_handlers = [];
    var tap_handler = new JSVXEventHandler(jsvx, this);
    tap_handler.tap = 1;
    this.event_handlers.push(tap_handler);
    this.event_handlers.push(new DefaultCameraControls(this));
}

// BUG? receives coordinates relative to canvas, not relative to
// layer.
function makeComputeManipulationPointXYPlane(z)
{
    return makeComputeManipulationPointPlane(0, 0, 1, z);
}

// pick the point nearest the plane Ax+By+Cz=D
function makeComputeManipulationPointPlane(A, B, C, D)
{
    return function(x, y) {
        // start of ray
        var r0 = gluUnproject([ x, this.renderinfo.viewport[3] - y, 0],
                              this.renderinfo.viewMatrix, this.renderinfo.projectionMatrix,
                              this.renderinfo.viewport);

        // direction of ray
        var r1 = gluUnproject([ x, this.renderinfo.viewport[3] - y, 1],
                              this.renderinfo.viewMatrix, this.renderinfo.projectionMatrix,
                              this.renderinfo.viewport);

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

VwLayer.prototype.onkeypress = function(e) {
    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onkeypress(e))
            return true;
    }
    return false;
}

VwLayer.prototype.onkeydown = function(e) {
    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onkeydown(e))
            return true;
    }
    return false;
}

VwLayer.prototype.onkeyup = function(e) {
    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onkeyup(e))
            return true;
    }
    return false;
}

VwLayer.prototype.ontouchstart = function(e) {
    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].ontouchstart(e))
            return true;
    }
    return false;
}

VwLayer.prototype.ontouchmove = function(e) {
    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].ontouchmove(e))
            return true;
    }
    return false;
}

VwLayer.prototype.ontouchend = function(e) {
    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].ontouchend(e))
            return true;
    }
    return false;
}

VwLayer.prototype.onmousewheel = function(e) {
    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onmousewheel(e))
            return true;
    }

    return false;
}

VwLayer.prototype.onmousedown = function(e) {

    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onmousedown(e))
            return true;
    }

    return false;
};

// XXX should be "onclick"?
VwLayer.prototype.onmouseclick = function(e) {

    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onmouseclick(e))
            return true;
    }

    return false;
};

VwLayer.prototype.onmouseup = function(e) {

    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onmouseup(e))
            return true;
    }

    return false;
};

VwLayer.prototype.onmousemove = function(e) {

    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onmousemoved(e))
            return true;
    }

    return false;
};

VwLayer.prototype.onorientationchange = function(e) {

    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onorientationchange) {
            if (this.event_handlers[i].onorientationchange(e))
                return true;
        }
    }

    return false;
};

VwLayer.prototype.onresize = function(e) {

    for (var i = 0; i < this.event_handlers.length; i++) {
        if (this.event_handlers[i].onresize) {
            if (this.event_handlers[i].onresize(e))
                return true;
        }
    }

    return false;
};


// this function operates in GL y coordinates (Y increases going up).
function computePanJacobian(xyz, P, viewport, eye, c, up, dir1, dir2)
{
    var eps = 0.0001;

    var M0 = lookat_to_matrix(eye, c, up);
    var w0 = gluProject(xyz, M0, P, viewport);

    var M1 = lookat_to_matrix(vec_add(eye, vec_scale(dir1, eps)),
                              vec_add(c, vec_scale(dir1, eps)),
                              up);
    var w1 = gluProject(xyz, M1, P, viewport);

    var M2 = lookat_to_matrix(vec_add(eye, vec_scale(dir2, eps)),
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
        var M = lookat_to_matrix(eye, lookat, up);

        var winpos0 = gluProject(xyz, M, P, viewport);

        var err = mat_create(2, 1, [ winx - winpos0[0], winy - winpos0[1] ]);

        var lookVec = vec_normalize(vec_subtract(lookat, eye));
        var left = vec_crossproduct(up, lookVec);

        var dir1 = vec_copy(up);
        var dir2 = vec_copy(left);

        if (true) { // 2.5D
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

VwLayer.prototype.getBuffer = function(name)
{
    for (var i = 0; i < this.buffers.length; i++) {
        vb = this.buffers[i];
        if (vb.name == name)
            return vb;
    }

    var vb = new VwBuffer(this, name);
    vb.visible = true;
    this.buffers.push(vb);

    for (var i = 0; i < this.on_buffer_callbacks.length; i++) {
        this.on_buffer_callbacks[i](this, vb);
    }

    return vb;
};

VwLayer.prototype.makeRenderInfo = function()
{
    var canvas_width = this.canvas.getWidth(), canvas_height = this.canvas.getHeight();

    var mtime                = new Date().getTime();
    var renderinfo           = new Object();
    renderinfo.mtime         = mtime;
    renderinfo.viewport      = this.layerViewportComputer.compute(canvas_width, canvas_height, mtime);

    renderinfo.elu           = this.viewComputer.compute(mtime);
    renderinfo.eye           = renderinfo.elu.eye;
    renderinfo.lookat        = renderinfo.elu.lookat;
    renderinfo.up            = renderinfo.elu.up;
    renderinfo.viewMatrix    = renderinfo.elu.V;

    renderinfo.projinfo      = this.projectionComputer.compute(renderinfo.viewport[2], renderinfo.viewport[3], mtime);
    renderinfo.projectionMatrix = renderinfo.projinfo.P;
    return renderinfo;
}

VwLayer.prototype.draw = function()
{
    console.log("layer draw");

    var gl = this.canvas.gl;

    stable_sort(this.buffers, function(a, b) { return a.draworder - b.draworder });
//    this.buffers.sort(function(a, b) { return a.draworder - b.draworder });

    this.renderinfo = this.makeRenderInfo();

    var stack = new VwStack(this.renderinfo.projinfo.P,
                            this.renderinfo.elu.V,
                            mat_identity(4));

    var viewport = this.renderinfo.viewport;

    gl.scissor(viewport[0], viewport[1], viewport[2], viewport[3]);
    gl.viewport(viewport[0], viewport[1], viewport[2], viewport[3]);

    gl.clearColor(this.background_rgba[0], this.background_rgba[1], this.background_rgba[2], this.background_rgba[3]);

    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    for (var i = 0; i < this.buffers.length; i++) {
        if (this.visible && this.buffers[i].visible)
            this.buffers[i].draw(stack);
    }

    var eps = 0.01;
    if (!this.renderinfo.elu.done || !this.renderinfo.projinfo.done || !this.renderinfo.viewport.done)
        this.canvas.onDirty();
};

/////////////////////////////////////////////////////////
function VwBuffer(layer, name)
{
    this.layer = layer;
    this.name = name;
    this.draworder = 1;
    this.back = [];
    this.front = [];
}

VwBuffer.prototype.addBack = function(obj)
{
    this.back.push(obj);
};

VwBuffer.prototype.swap = function()
{
    this.front = this.back;
    this.back = [];

    this.layer.canvas.onDirty();
};

VwBuffer.prototype.draw = function(stack)
{
    var gl = this.layer.canvas.gl;

    console.log(this.draworder);

    for (var i = 0; i < this.front.length; i++)
        this.front[i].draw(this.layer.canvas, this.layer, stack);
};

function lookat_to_matrix(eye, c, up)
{
    up = vec_normalize(up);
    var f = vec_normalize(vec_subtract(c, eye));

    var s = vec_crossproduct(f, up);
    var u = vec_crossproduct(s, f);

    var M = mat_create(4, 4, [ s[0], s[1], s[2], 0,
                               u[0], u[1], u[2], 0,
                               -f[0], -f[1], -f[2], 0,
                               0, 0, 0, 1]);

    var T = mat_create(4, 4, [ 1, 0, 0, -eye[0],
                               0, 1, 0, -eye[1],
                               0, 0, 1, -eye[2],
                               0, 0, 0, 1 ]);

    return mat_multiply(M, T);
};

function gluUnproject(winxyz, M, P, viewport)
{
    var invPM = mat_inverse(mat_multiply(P, M));

    var v = mat_create(4, 1, [ 2*(winxyz[0]-viewport[0]) / viewport[2] - 1,
                               2*(winxyz[1]-viewport[1]) / viewport[3] - 1,
                               2*winxyz[2] - 1,
                               1 ]);

    var objxyzh = mat_multiply(invPM, v);

    return [ objxyzh[0] / objxyzh[3],
             objxyzh[1] / objxyzh[3],
             objxyzh[2] / objxyzh[3] ];
}

function gluProject(xyz, M, P, viewport)
{
    var xyzh = mat_create(4, 1, [ xyz[0], xyz[1], xyz[2], 1]);

    var p = mat_multiply(mat_multiply(P, M), xyzh);

    p[0] = p[0] / p[3];
    p[1] = p[1] / p[3];
    p[2] = p[2] / p[3];

    return mat_create(3, 1, [ viewport[0] + viewport[2]*(p[0]+1)/2.0,
                              viewport[1] + viewport[3]*(p[1]+1)/2.0,
                              (viewport[2] + 1)/2.0 ]);
}

function gluPerspective(fovy_degrees, aspect, znear, zfar)
{
    var f = 1.0 / Math.tan((fovy_degrees / 2) * (Math.PI / 180));

    return mat_create(4, 4, [ f/aspect, 0, 0, 0,
                              0,        f, 0, 0,
                              0,        0, (zfar+znear)/(znear-zfar), 2*zfar*znear / (znear-zfar),
                              0, 0, -1, 0 ]);
}

function glOrtho(left, right, bottom, top, znear, zfar)
{
    return mat_create(4, 4, [ 2/(right-left), 0, 0, -(right+left)/(right-left),
                              0, 2/(top-bottom), 0, -(top+bottom)/(top-bottom),
                              0, 0, -2/(zfar-znear), -(zfar+znear)/(zfar-znear),
                              0, 0, 0, 1 ]);
}

// uses camera.[eye|lookat|up][01] to fill in renderinfo.[eye|lookat|up] and renderinfo.perspectiveness
function interpolate_orbit(camera, mtime, renderinfo)
{
    var lookdistance = vec_magnitude(vec_subtract(camera.eye0, camera.lookat0));

    var alpha1 = (camera.mtime1 == camera.mtime0) ? 1 : (mtime - camera.mtime0) / (camera.mtime1 - camera.mtime0);
    var alpha0 = 1.0 - alpha1;

    for (var i = 0; i < 3; i++) {
        renderinfo.eye[i]    = camera.eye0[i]   *alpha0 + camera.eye1[i]   *alpha1;
        renderinfo.lookat[i] = camera.lookat0[i]*alpha0 + camera.lookat1[i]*alpha1;
        renderinfo.up[i]     = camera.up0[i]    *alpha0 + camera.up1[i]    *alpha1;
    }

    // constrain distance between eye and lookat to be the same distance it was before.
    var lookdir = vec_normalize(vec_subtract(renderinfo.eye, renderinfo.lookat));
    renderinfo.eye = vec_add(renderinfo.lookat, vec_scale(lookdir, lookdistance));

    renderinfo.perspectiveness = camera.perspectiveness0*alpha0 + camera.perspectiveness1*alpha1;
}

function interpolate_linear(camera, mtime, renderinfo)
{
    var alpha1 = (camera.mtime1 == camera.mtime0) ? 1 : (mtime - camera.mtime0) / (camera.mtime1 - camera.mtime0);
    var alpha0 = 1.0 - alpha1;

    for (var i = 0; i < 3; i++) {
        renderinfo.eye[i]    = camera.eye0[i]   *alpha0 + camera.eye1[i]   *alpha1;
        renderinfo.lookat[i] = camera.lookat0[i]*alpha0 + camera.lookat1[i]*alpha1;
        renderinfo.up[i]     = camera.up0[i]    *alpha0 + camera.up1[i]    *alpha1;
    }
}

/////////////////////////////////////////////////////////
function VwObject()
{

}

VwObject.prototype.draw = function(canvas, layer, stack)
{
};

/////////////////////////////////////////////////////////
function VwChain()
{
    VwObject.call(this);

    this.objects = Array.prototype.slice.call(arguments); // var-args
}

VwChain.prototype = Object.create(VwObject.prototype);

VwChain.prototype.add = function(obj)
{
    this.objects.push(obj);
}

VwChain.prototype.draw = function(canvas, layer, stack)
{
    stack.M.pushM();

    for (var i = 0; i < this.objects.length; i++) {
        var obj = this.objects[i];

        if (Array.isArray(obj)) {
            stack.M.multiply(obj);
            continue;
        }

        obj.draw(canvas, layer, stack);

    }

    stack.M.pop();
};

function JSVXEventHandler(jsvx, layer)
{
    this.jsvx = jsvx;
    this.layer = layer;
    this.tap = 0;
    this.tapStart = 0;
    this.currX = 0;
    this.currY = 0;
    this.cacheX = 0;
    this.cacheY = 0;
}

JSVXEventHandler.prototype.fillInEvent = function(e, code) {
    let douts = new DataOutput(1024);
    douts.write_u32(0x1186712a); // magic
    douts.write_u32(code);
    douts.write_u64(e.timeStamp * 1000 );    // utime on client
    douts.write_string_u32(this.layer.name);

    for (let i = 0; i < 4; i++)
        douts.write_f32(this.renderinfo.viewport[i]);

    for (let i = 0; i < 16; i++)
        douts.write_f64(this.renderinfo.projectionMatrix[i]);

    for (let i = 0; i < 16; i++)
        douts.write_f64(this.renderinfo.viewMatrix[i]);

    let flags = 0;
    if (e.altKey)
        flags |= 1;
    if (e.ctrlKey)
        flags |= 2;
    if (e.shiftKey)
        flags |= 4;
    douts.write_u32(flags);

    return douts;
}

JSVXEventHandler.prototype.onkeydown = function(e) {

    if(this.tap) return;
    var douts = this.fillInEvent(e, 1010); // KEY_DOWN

    douts.write_u32(e.keyCode);

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
};

JSVXEventHandler.prototype.onkeypress = function(e) {

    if(this.tap) return;
    var douts = this.fillInEvent(e, 1011); // KEY_PRESS

    douts.write_u32(e.keyCode);

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
};

JSVXEventHandler.prototype.onkeyup = function(e) {

    if(this.tap) return;
    var douts = this.fillInEvent(e, 1012); // KEY_UP

    douts.write_u32(e.keyCode);

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
};

JSVXEventHandler.prototype.ontouchstart = function(e) {

    if(this.tap){


        this.tapStart = 1;
        this.cacheX = this.currX = e.xys[0][0];
        this.cacheY = this.currY = e.xys[0][1];


        var ots = this;

        setTimeout(function (){
            if ((ots.cacheX == ots.currX) && !ots.tapStart && (ots.cacheY == ots.currY)) {

                var douts = ots.fillInEvent(e, 1023); // TOUCH_TAP

                douts.write_f32(e.xys[0][0]);
                douts.write_f32(e.xys[0][1]);
                douts.write_u32(e.touches.length);
                douts.write_u32(e.changedTouches[0].identifier);

                if (jsvx.ws.readyState == jsvx.ws.OPEN)
                    jsvx.ws.send(douts.data.subarray(0, douts.datapos));

            }
        }
                   ,200);


        return;
    }

    var douts = this.fillInEvent(e, 1020); // TOUCH_START

    douts.write_f32(e.xys[0][0]);
    douts.write_f32(e.xys[0][1]);
    douts.write_u32(e.touches.length);
    douts.write_u32(e.changedTouches[0].identifier);

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
};

JSVXEventHandler.prototype.ontouchmove = function(e) {

    if(this.tap){
        //this.currX = e.xys[0][0];
        //this.currY = e.xys[0][1];
        return;
    }

    for (var i = 0; i < e.xys.length; i++) {

        var douts = this.fillInEvent(e, 1021); // TOUCH_MOVE

        douts.write_f32(e.xys[i][0]);
        douts.write_f32(e.xys[i][1]);
        douts.write_u32(e.touches.length);
        douts.write_u32(e.changedTouches[i].identifier);

        if (jsvx.ws.readyState == jsvx.ws.OPEN)
            jsvx.ws.send(douts.data.subarray(0, douts.datapos));
    }
};

JSVXEventHandler.prototype.ontouchend = function(e) {

    if(this.tap){
        this.tapStart = 0;
        return;
    }

    var douts = this.fillInEvent(e, 1022); // TOUCH_END

    douts.write_f32(e.xys[0][0]);
    douts.write_f32(e.xys[0][1]);
    douts.write_u32(e.touches.length);
    douts.write_u32(e.changedTouches[0].identifier);

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
};

JSVXEventHandler.prototype.onorientationchange = function(e) {
    if(this.tap) return;
    var douts = this.fillInEvent(e, 1040); //

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
}

JSVXEventHandler.prototype.onresize = function(e) {
    if(this.tap) return;
    var douts = this.fillInEvent(e, 1040); //

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
}

JSVXEventHandler.prototype.onmousewheel = function(e) {
    if(this.tap) return;

    var douts = this.fillInEvent(e, 1005); //

    douts.write_f32(e.wheelDelta); // XXX Untested

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
};

JSVXEventHandler.prototype.onmousedown = function(e) {
    if(this.tap) return;
    this.mousedowntime = new Date().getTime();
    this.mousedownpos = [ e.xy[0], e.xy[1] ];

    var douts = this.fillInEvent(e, 1001); // MOUSE_DOWN

    douts.write_f32(e.xy[0]);
    douts.write_f32(e.xy[1]);
    douts.write_u8(e.button);

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));

    return true;
};

JSVXEventHandler.prototype.onmouseclick = function(e) {
    if(this.tap) return;
    var time = new Date().getTime();

    // don't fire click on a drag.
    if (time - this.mousedowntime > 250)
        return;

    var dist = Math.sqrt(Math.pow(this.mousedownpos[0] - e.xy[0], 2) + Math.pow(this.mousedownpos[1] - e.xy[1], 2));
    if (dist > 10)
        return;

    var douts = this.fillInEvent(e, 1004); // MOUSE_CLICKED

    douts.write_f32(e.xy[0]);
    douts.write_f32(e.xy[1]);
    douts.write_u8(e.button);

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
};

JSVXEventHandler.prototype.onmouseup = function(e) {
    if(this.tap) return;

    var douts = this.fillInEvent(e, 1003); // MOUSE_UP

    douts.write_f32(e.xy[0]);
    douts.write_f32(e.xy[1]);
    douts.write_u8(e.button);

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));
};

JSVXEventHandler.prototype.onmousemoved = function(e) {
    if(this.tap) return;

    var douts = this.fillInEvent(e, 1002); // MOUSE_MOVED

    douts.write_f32(e.xy[0]);
    douts.write_f32(e.xy[1]);

    if (jsvx.ws.readyState == jsvx.ws.OPEN)
        jsvx.ws.send(douts.data.subarray(0, douts.datapos));

};
