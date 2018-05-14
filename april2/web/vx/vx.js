"use strict";

function VxCanvas(canvasid)
{
    // we store resources here
    this.programs = new Object();
    this.attributes = new Object();
    this.indices = new Object(); // index buffers
    this.textures = new Object();

    //////////////////////////////////////////////////
    // set up the websocket
    var mode = "ws://";
    if (document.location.href.match('/https/g'))
        mode = "wss://";

    this.ws = new WebSocket(mode + document.location.host + "/webvx", [ canvasid ]);
    this.ws.binaryType = "arraybuffer";

    var self = this;
    this.ws.onopen = function() { console.log("VxCanvas: web socket opened."); };
    this.ws.onclose = function() { console.log("VxCanvas: web socket closed."); };
    this.ws.onmessage = function(e) { self.onMessage(e); };

    this.lastReceivedMessageTime = Date.now();
    this.lastSentMessageType = "";
    this.lastSentMessageTime = 0;

    //////////////////////////////////////////////////
    // set up the GL canvas
    this.canvasid = canvasid;
    this.glcanvas = document.getElementById(canvasid);
    this.glcanvas.focus();
    this.gl = this.glcanvas.getContext("webgl", {preserveDrawingBuffer: true });
    if (this.gl == null)
        this.gl = this.glcanvas.getContext("experimental-webgl");

    //    this.gl = WebGLDebugUtils.makeDebugContext(this.gl, throwOnGLError, logGLCall);

    var gl = this.gl;

    // called when there is a new buffer/layer or draw order is changed.
    this.onBufferLayerDrawOrder = [ ];

    this.glcanvas.addEventListener('contextmenu', function(e) { e.preventDefault(); });
    this.glcanvas.oncontextmenu = function() { return false; };

    this.glcanvas.focus();

    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LESS);

    gl.enable(gl.BLEND);
    gl.enable(gl.SCISSOR_TEST);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

    // .canvas, .name, .buffers, .draworder, .event_handlers
    this.layers = [];

    /*
    this.glcanvas.addEventListener('DOMMouseScroll',
                                   function(e) {
                                       e.wheelDelta = Math.max(-1, Math.min(1, (e.wheelDelta || -e.detail))); //FF hack
                                       _vw.onmousewheel(e);
                                   });
*/
    this.glcanvas.onmousedown  = function(e) { return self.dispatchToLayer("onmousedown", e); };
    this.glcanvas.onmouseup    = function(e) { return self.dispatchToLayer("onmouseup", e); };
    this.glcanvas.onmousemove  = function(e) { return self.dispatchToLayer("onmousemove", e); };
    this.glcanvas.onwheel      = function(e) { return self.dispatchToLayer("onmousewheel", e); };
    this.glcanvas.onclick      = function(e) { return self.dispatchToLayer("onmouseclick", e); };

    this.glcanvas.ontouchstart = function(e) { return self.dispatchToLayer("ontouchstart", e); };
    this.glcanvas.ontouchmove  = function(e) { return self.dispatchToLayer("ontouchmove", e); };
    this.glcanvas.ontouchend   = function(e) { return self.dispatchToLayer("ontouchend", e); };

    this.glcanvas.onkeypress   = function(e) { return self.dispatchToLayer("onkeypress", e); };
    this.glcanvas.onkeydown    = function(e) { return self.dispatchToLayer("onkeydown", e); };
    this.glcanvas.onkeyup      = function(e) { return self.dispatchToLayer("onkeyup", e); };

    var oldfn = document.onmouseup;
    document.onmouseup = function(e) { self.dispatchToLayer("onmouseup", e); if (oldfn) return oldfn(e); };

    this.requestors = [ ];
    this.dirty = false;

    window.setInterval(function() {
//        console.log(self.requestors.length);
        if (self.dirty || self.requestors.length > 0) {
            self.dirty = false;

            //this.draw();
            window.requestAnimationFrame(function() { self.draw(); });
        }
    }, 30);

    window.onorientationchange = function(e) {
        var douts = new DataOutput(128);

        douts.write_u32(0x1186712a); // magic
        douts.write_u32(VX_EVENT_CANVAS_CHANGE);
        douts.write_u32(self.glcanvas.width);
        douts.write_u32(self.glcanvas.height);

        self.sendMessage("oncanvaschange", douts);
    };

    window.onresize = function(e) {
        var douts = new DataOutput(128);

        douts.write_u32(0x1186712a); // magic
        douts.write_u32(VX_EVENT_CANVAS_CHANGE);
        douts.write_u32(self.glcanvas.width);
        douts.write_u32(self.glcanvas.height);

        self.sendMessage("oncanvaschange", douts);
    };
};

VxCanvas.prototype.notifyBufferLayerDrawOrder = function()
{
    for (var i = 0; i < this.onBufferLayerDrawOrder.length; i++)
        this.onBufferLayerDrawOrder[i](this);
};

VxCanvas.prototype.requestRedraw = function()
{
    this.dirty = true;
};

// A camera animating a transition might request continous redraws
// during the transition. When the animation is complete, they should
// call End(). The requestor is a string that identifies WHO has
// requested continous redraws. There is internally a set of
// requestors.
VxCanvas.prototype.requestContinuousRedraw = function(requestor)
{
    // don't add multiple times.
    for (var i = 0; i < this.requestors.length; i++)
        if (this.requestors[i] == requestor)
            return;

    this.requestors.push(requestor);
}

VxCanvas.prototype.requestContinuousRedrawEnd = function(requestor)
{
    var newrequestors = [ ];
    for (var i = 0; i < this.requestors.length; i++)
        if (this.requestors[i] != requestor)
            newrequestors.push(this.requestors[i]);

    this.requestors = newrequestors;
}

VxCanvas.prototype.getLayer = function(name)
{
    for (var layeridx = this.layers.length - 1; layeridx >= 0; layeridx--) {
        var layer = this.layers[layeridx];
        if (layer.name == name)
            return layer;
    }

    var layer = { };
    layer.vc = this;
    layer.name = name;
    layer.buffers = [ ];
    layer.draworder = 0;
    layer.event_handlers = [];
    layer.visible = true;

    var cameraControls = new DefaultCameraControls(layer);

    layer.setup_handlers = [ ];
    layer.setup_handlers.push(cameraControls);

    var mtime = new Date().getTime();

    this.layers.push(layer);
    this.notifyBufferLayerDrawOrder();

    // force a draw, mostly so that we populate the layer's
    // renderinfo.
    this.draw();

    layer.event_handlers.push(new VxRemoteEventHandler());
    layer.event_handlers.push(cameraControls);

    return layer;
}

VxCanvas.prototype.computeEventCoordinates = function(clientX, clientY)
{
    var devicePixelRatio = window.devicePixelRatio || 1;

    var xy = [ (clientX - this.glcanvas.offsetLeft) * devicePixelRatio,
               (clientY - this.glcanvas.offsetTop) * devicePixelRatio ];

    return xy;
};

VxCanvas.prototype.dispatchToLayer = function(type, e)
{
    var targetLayer = null;

    if (type.startsWith("ontouch")) {
        e.preventDefault(); // Suppress taps
    }

    if (type.startsWith("onkey")) {
        if (this.keyboardFocusLayer == null) {
            this.keyboardFocusLayer = this.layers[0];
        }

        targetLayer = this.keyboardFocusLayer;

        // prevent browser from grabbing the tab and moving the focus
        // somewhere else.
        if (e.keyCode == 9)
            e.preventDefault();
    }

    if (type.startsWith("onmouse")) {
        e.xy = this.computeEventCoordinates(e.clientX, e.clientY);
    } else if (type.startsWith("ontouch")) {
        e.xy = this.computeEventCoordinates(e.changedTouches[0].clientX,
                                            e.changedTouches[0].clientY);
    }

    if (type == "onmousedown" || type == "onmousewheel" || type == "onmouseclick" || type == "ontouchstart") {
        // process from "topmost" to "bottommost"
        for (var layeridx = this.layers.length - 1; layeridx >= 0; layeridx--) {

            var layer = this.layers[layeridx];
            if (!layer.visible)
                continue;

            if (e.xy[0] >= layer.viewport[0] &&
                e.xy[0] < layer.viewport[0] + layer.viewport[2] &&
                (this.glcanvas.height - e.xy[1]) >= layer.viewport[1] &&
                (this.glcanvas.height - e.xy[1]) < layer.viewport[1] + layer.viewport[3]) {

                targetLayer = layer;
                if (true || type == "onmousedown") {
                    this.keyboardFocusLayer = layer;
                    this.mouseDownLayer = layer;
                }
                break;
            }
        }
    } else if (type.startsWith("onmouse") || type.startsWith("ontouch")) {
        // mouseup event always goes to the layer that got the
        // mousedown event.  NB: mouseup events can occur outside of
        // the layer's viewport when someone drags off-layer.
        targetLayer = this.mouseDownLayer;
    }

    var result = null;

    if (targetLayer) {
        stable_sort(targetLayer.event_handlers, function(a, b) { return a.eventorder - b.eventorder });

        for (var handleridx = 0; handleridx < targetLayer.event_handlers.length; handleridx++) {
            if (targetLayer.event_handlers[handleridx].onEvent(targetLayer, type, e)) {
                result = true;
                break;
            }
        }
    }

    if (type == "onmouseup" || type == "ontouchup")
        this.mouseDownLayer = null;

    return result;
};

function draworder_sort(a, b)
{
    if (a.draworder != b.draworder)
        return a.draworder - b.draworder;
    return a.name.localeCompare(b.name);
}

VxCanvas.prototype.draw = function()
{
    stable_sort(this.layers, draworder_sort);

    var gl = this.gl;
    var mtime = new Date().getTime();

    // this is here primarily in case no layer draws to an area.
    gl.scissor(0, 0, this.glcanvas.width, this.glcanvas.height);
    gl.viewport(0, 0, this.glcanvas.width, this.glcanvas.height);
    gl.clearColor(0.1, 0.1, 0.1, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // from bottom-most to top-most
    for (var layeridx = 0; layeridx < this.layers.length; layeridx++) {
        var layer = this.layers[layeridx];

        if (!layer.visible)
            continue;

        // before every draw, we call the setup handlers, which are responsible
        // for configuring:
        //
        // layer.viewport
        // layer.eye
        // layer.lookat
        // layer.up
        // layer.V                elu_to_matrix(eye, lookat, up)
        // layer.P                projection matrix
        // layer.background

        stable_sort(layer.setup_handlers, function(a, b) { a.setuporder - b.setuporder });

        for (var handleridx = 0; handleridx < layer.setup_handlers.length; handleridx++)
            layer.setup_handlers[handleridx].setupLayer(layer, mtime, this.glcanvas.width, this.glcanvas.height);

        gl.scissor(layer.viewport[0], layer.viewport[1], layer.viewport[2], layer.viewport[3]);
        gl.viewport(layer.viewport[0], layer.viewport[1], layer.viewport[2], layer.viewport[3]);

        gl.clearColor(layer.background[0],
                      layer.background[1],
                      layer.background[2],
                      layer.background[3]);

        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

        var stack = { P: new MatrixStack(layer.P),
                      V: new MatrixStack(layer.V),
                      M: new MatrixStack(mat_identity(4)),
                      depth_test: new EnableStack(1)
                    };

        stable_sort(layer.buffers, draworder_sort);

        for (var i = 0; i < layer.buffers.length; i++) {
            if (layer.visible && layer.buffers[i].visible)
                layer.buffers[i].draw(vc, layer, stack);
        }
    }

    // send draw notifications back to the vx app
    if (true) {
        var douts = new DataOutput(128);

        douts.write_u32(0x1186712a); // magic
        douts.write_u32(1000); // ON_DRAW

        this.sendMessage("draw", douts);
    }
}

// type is descriptive... if we are rate limiting, we might want to
// reduce the number of MOUSEMOVED messages, but we don't want to
// eliminate the MOUSEDOWN. Tagging each type allows us to be more
// clever.
VxCanvas.prototype.sendMessage = function(type, douts)
{
//    console.log("sendMessage: "+type);

    var time = Date.now(); // in ms.

    // XXX it would be better if, when we DO cull a message, we
    // remember it so that if it is the last message of that type for
    // a long time, we ultimately transmit it anyway.

    // a very simple policy: cull back-to-back messages of the same
    // type if they are within a small period of time.
    if (type == this.lastSentMessageType && (time - this.lastSentMessageTime < 5)) {
//        console.log("culling message of type " + type);
        return;
    }

    if (this.ws.readyState == this.ws.OPEN)
        this.ws.send(douts.data.subarray(0, douts.datapos));
    else
        console.log("message not sent; ws in wrong state");

    this.lastSentMessageType = type;
    this.lastSentMessageTime = time;
};

var VX_MESSAGE_BUFFER_REDRAW = 1,
    VX_MESSAGE_BUFFER_DESTROY = 24,
    VX_MESSAGE_CANVAS_READ_PIXELS = 2,
    VX_MESSAGE_CANVAS_SET_SIZE = 3,
    VX_MESSAGE_CANVAS_SET_TITLE = 4,
    VX_MESSAGE_LAYER_ENABLE_CAMERA_CONTROLS = 5,
    VX_MESSAGE_LAYER_SET_DRAW_ORDER = 6,
    VX_MESSAGE_LAYER_SET_BACKGROUND_COLOR = 7,
    VX_MESSAGE_LAYER_SET_POSITION = 8,
    VX_MESSAGE_LAYER_SET_ELU = 9,
    VX_MESSAGE_NOP = 10,
    VX_MESSAGE_DEBUG_MESSAGE = 11,
    VX_MESSAGE_DEFINE_PROGRAM = 12,
    VX_MESSAGE_UNDEFINE_PROGRAM = 13,
    VX_MESSAGE_DEFINE_VERTEX_ATTRIBUTE = 14,
    VX_MESSAGE_UNDEFINE_VERTEX_ATTRIBUTE = 15,
    VX_MESSAGE_DEFINE_INDEX_ARRAY = 16,
    VX_MESSAGE_UNDEFINE_INDEX_ARRAY = 17,
    VX_MESSAGE_DEFINE_TEXTURE = 18,
    VX_MESSAGE_UNDEFINE_TEXTURE = 19,
    VX_MESSAGE_DEFINE_NAMED_MATRIX = 20,
    VX_MESSAGE_UNDEFINE_NAMED_MATRIX = 21,
    VX_MESSAGE_LAYER_SET_CAMERA_MODE = 22,
    VX_MESSAGE_CANVAS_ECHO = 23;

var VX_SERIALIZER_MODEL_PUSH = 1,
    VX_SERIALIZER_MODEL_POP = 2,
    VX_SERIALIZER_MODEL_MULTIPLY = 3,
    VX_SERIALIZER_PIXCOORD_PUSH = 4,
    VX_SERIALIZER_PIXCOORD_POP = 5,
    VX_SERIALIZER_DEPTH_TEST_PUSH = 6,
    VX_SERIALIZER_DEPTH_TEST_POP = 7,
    VX_SERIALIZER_EXECUTE_PROGRAM = 100;

var VX_EVENT_CANVAS_DRAW = 1000,
    VX_EVENT_MOUSE_DOWN = 1001, VX_EVENT_MOUSE_MOVED = 1002, VX_EVENT_MOUSE_UP = 1003, VX_EVENT_MOUSE_CLICKED = 1004,
    VX_EVENT_MOUSE_WHEEL = 1005,
    VX_EVENT_KEY_DOWN = 1010, VX_EVENT_KEY_PRESSED = 1011, VX_EVENT_KEY_UP = 1012,
    VX_EVENT_TOUCH_START = 1020, VX_EVENT_TOUCH_MOVE = 1021, VX_EVENT_TOUCH_END = 1022, VX_EVENT_TOUCH_TAP = 1023,
    VX_EVENT_CANVAS_ECHO = 1030 , VX_EVENT_CANVAS_CHANGE = 1040,
    VX_EVENT_READPIXELS = 2000;

/////////////////////////////////////////////////////
// has arrays added to it when we parse the opcode stream...

function VxBuffer(name, data)
{
    this.name = name;
    this.data = data;
    this.visible = true;
    this.draworder = 0;
}

VxBuffer.prototype.draw = function(vc, layer, stack)
{
    if (!this.visible)
        return;

    var din = new DataInput(this.data);
    var gl = vc.gl;

    while (din.has_more()) {
        var opcode = din.read_u8();

        switch (opcode)
        {
            case 0: {
                return;
            }

            case VX_SERIALIZER_MODEL_PUSH: {
                stack.M.push();
                break;
            }

            case VX_SERIALIZER_MODEL_POP: {
                stack.M.pop();
                break;
            }

            case VX_SERIALIZER_MODEL_MULTIPLY: {
                var M = mat_create(4,4);
                for (var i = 0; i < 16; i++)
                    M[i] = din.read_f64();

                stack.M.multiply(M);
                break;
            }

            case VX_SERIALIZER_PIXCOORD_PUSH: {
                // when setting up the transformation, by how much
                // should we translate the origin as a multiple of
                // the viewport size?  e.g., scale=1 means move
                // the origin by the full width of the viewport.
                var widthscale = din.read_f32();
                var heightscale = din.read_f32();

                // when 0, 1 = 1 pixel
                // when 1, 1 = viewport width
                // when 2, 1 = viewport height.
                // when 3, 1 = min(width, height)
                // when 4, 1 = max(width, height)
                var scale_mode = din.read_u8();

                stack.P.push();
                stack.V.push();
                stack.M.push();

                var width = layer.viewport[2];
                var height = layer.viewport[3];

                stack.P.set(glOrtho(0, width, 0, height, -1E8, 1E8));
                stack.V.set(mat_identity(4));
                var T = mat_translate(width*widthscale, height*heightscale, 0);
                var S = mat_scale(1);

                if (scale_mode == 1)
                    S = mat_scale(width);
                else if (scale_mode == 2)
                    S = mat_scale(height);
                else if (scale_mode == 3)
                    S = mat_scale(Math.min(width, height));
                else if (scale_mode == 4)
                    S = mat_scale(Math.max(width, height));

                stack.M.set(mat_multiply(T, S));

                break;
            }

            case VX_SERIALIZER_PIXCOORD_POP: {
                stack.P.pop();
                stack.V.pop();
                stack.M.pop();
                break;
            }

            case VX_SERIALIZER_DEPTH_TEST_PUSH: {
                var enable = din.read_u8();

                stack.depth_test.push();
                if (enable)
                    gl.enable(gl.DEPTH_TEST);
                else
                    gl.disable(gl.DEPTH_TEST);

                break;
            }

            case VX_SERIALIZER_DEPTH_TEST_POP: {
                stack.depth_test.pop();
                if (stack.depth_test.get())
                    gl.enable(gl.DEPTH_TEST);
                else
                    gl.disable(gl.DEPTH_TEST);
                break;
            }

            case VX_SERIALIZER_EXECUTE_PROGRAM: {
                var program_name = din.read_id();

                var glprogram = vc.programs[program_name];
                gl.useProgram(glprogram);

                var nuniforms = din.read_u8();

                for (var i = 0; i < nuniforms; i++) {
                    var uniform_name = din.read_string_u32();
                    var nrows = din.read_u8();
                    var ncols = din.read_u8(); // set to 1 for vector

                    var M = mat_create(nrows, ncols);
                    for (var j = 0; j < nrows*ncols; j++)
                        M[j] = din.read_f32();

                    if (uniform_name == "glLineWidth") {
                        gl.lineWidth(M[0]);
                        continue;
                    }

                    var location = getLocation(gl, glprogram, gl.getUniformLocation, uniform_name);

                    if (M.nrows == M.ncols) {
                        // matrices
                        if (M.nrows == 4)
                            gl.uniformMatrix4fv(location, false, mat_transpose(M));
                        else if (M.nrows == 3)
                            gl.uniformMatrix3fv(location, false, mat_transpose(M));
                        else if (M.nrows == 2)
                            gl.uniformMatrix2fv(location, false, mat_transpose(M));
                        else if (M.nrows == 1)
                            gl.uniform1fv(location, M);
                        else
                            console.log("unimplemented uniform matrix");
                    } else if (M.ncols == 1) {
                        if (M.nrows == 4)
                            gl.uniform4fv(location, M);
                        else if (M.nrows == 3)
                            gl.uniform3fv(location, M);
                        else if (M.nrows == 2)
                            gl.uniform2fv(location, M);
                        else
                            console.log("unimplemented uniform vector");

                    } else {
                        console.log("unimplemented uniform");
                    }
                }

                var nattrib = din.read_u8();
                var attribLocationsToUnbind = [];

                for (var i = 0; i < nattrib; i++) {
                    var attrib_name = din.read_string_u32();
                    var resource_name = din.read_id();

                    var location = getLocation(gl, glprogram, gl.getAttribLocation, attrib_name);
                    var attrib = vc.attributes[resource_name];

                    if (location != null) {
                        gl.bindBuffer(gl.ARRAY_BUFFER, attrib);
                        gl.enableVertexAttribArray(location);
                        gl.vertexAttribPointer(location, attrib.ndim, attrib.type, false, 0, 0);
                        attribLocationsToUnbind.push(location);
                    }
                }

                var ntextures = din.read_u8();
                for (var i = 0; i < ntextures; i++) {
                    var sampler_name = din.read_string_u32();
                    var texture_name = din.read_id();

                    var location = getLocation(gl, glprogram, gl.getUniformLocation, sampler_name);
                    var texture = vc.textures[texture_name];

                    if (location != null) {
                        gl.activeTexture(gl.TEXTURE0 + i);
                        gl.bindTexture(gl.TEXTURE_2D, texture);
                        gl.uniform1i(location, i);
                    }
                }

                var Puniform = getLocation(gl, glprogram, gl.getUniformLocation, "VX_P");
                if (Puniform != null)
                    gl.uniformMatrix4fv(Puniform, false, mat_transpose(stack.P.get()));

                var Vuniform = getLocation(gl, glprogram, gl.getUniformLocation, "VX_V");
                if (Vuniform != null)
                    gl.uniformMatrix4fv(Vuniform, false, mat_transpose(stack.V.get()));

                var Muniform = getLocation(gl, glprogram, gl.getUniformLocation, "VX_M");
                if (Muniform != null)
                    gl.uniformMatrix4fv(Muniform, false, mat_transpose(stack.M.get()));

                var eyeuniform = getLocation(gl, glprogram, gl.getUniformLocation, "VX_eye");
                if (eyeuniform != null)
                    gl.uniform3fv(eyeuniform, layer.eye)

                var lookatuniform = getLocation(gl, glprogram, gl.getUniformLocation, "VX_lookat");
                if (lookatuniform != null)
                    gl.uniform3fv(lookatuniform, layer.lookat)

                var ndraws = din.read_u8();

                for (var i = 0; i < ndraws; i++) {
                    var uses_indices = din.read_u8();
                    var indices_name = null;
                    if (uses_indices)
                        indices_name = din.read_id();
                    else
                        indices_name = null;

                    var command = din.read_u32();
                    var first = din.read_u32();
                    var count = din.read_u32();
                    var type = gl.UNSIGNED_SHORT;

                    if (indices_name != null) {
                        var indices = vc.indices[indices_name];
                        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indices);
                        gl.drawElements(command, count, type, indices);
                    }  else {
                        gl.drawArrays(command, first, count);
                    }
                }

                // XXX Should we unbind attribute, textures, etc?
                for (var i = 0; i < attribLocationsToUnbind.length; i++) {
                    gl.disableVertexAttribArray(attribLocationsToUnbind[i]);
                }

                break;
            }

            default: {
                console.log("unknown opcode: "+opcode);
                break;
            }
        }
    }
}


function getLocation(gl, glprogram, glfunc, name)
{
    if (!("locations" in glprogram))
        glprogram["locations"] = new Object();

    var locs = glprogram["locations"];

    if (!(name in locs))
        locs[name] = glfunc.call(gl, glprogram, name);

    return locs[name];
}

/*    this.vc.on_layer_callbacks.push(function(layer) {
        console.log("new layer created '"+layer.name+"'");
    });
*/

    // Pass orienation change and resize events to all the layers
/*
  // XXX These are CANVAS events, not layer events.
  window.onorientationchange = function(e) {
        for (var i = 0; i < this.vc.layers.length; i++) {
            this.vc.layers[i].onorientationchange(e);
        }
    }

    window.addEventListener("resize", function(e) {
        for (var i = 0; i < this.vc.layers.length; i++) {
            this.vc.layers[i].onresize(e);
        }
    });
*/

/*
    this.lastReceivedMessageTime = Date.now();
    this.timeout = setInterval(function() {
        var time_since = Date.now() - this.lasMmessage;
        if ((time_since) > 2000) {
            document.getElementById("timeoutwindow").style.display = "block";
            document.getElementById("timeoutwindowtext").innerText = "Disconnected for " + Math.floor((time_since) / 1000) + "s";
        }
        else {
            document.getElementById("timeoutwindow").style.display = "none";
        }

    }.bind(this), 500);
*/

/*
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
    var douts = new DataOutput(1024);
    douts.write_u32(0x1186712a); // magic
    douts.write_u32(code);
    douts.write_u64(e.timeStamp * 1000 );    // utime on client
    douts.write_string_u32(this.layer.name);

    for (var i = 0; i < 4; i++)
        douts.write_f32(this.renderinfo.viewport[i]);

    for (var i = 0; i < 16; i++)
        douts.write_f64(this.renderinfo.projectionMatrix[i]);

    for (var i = 0; i < 16; i++)
        douts.write_f64(this.renderinfo.viewMatrix[i]);

    var flags = 0;
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
*/

VxCanvas.prototype.onMessage = function(e)
{
    var gl = this.gl;
    var din = new DataInput(new Uint8Array(e.data));

    var magic = din.read_u32();
    if (magic != 0x124578ab)
        return;

    this.lastReceivedMessageTime = Date.now();

    while (din.has_more()) {

        var opcode = din.read_u8();

        if (opcode == 0)
            break;

        switch (opcode)
        {
            case VX_MESSAGE_NOP: {
                break;
            }

            case VX_MESSAGE_DEBUG_MESSAGE: {
                var s = din.read_string_u32();
                console.log(s);
                break;
            }

            case VX_MESSAGE_DEFINE_PROGRAM: {
                var name = din.read_id();
                var vertex_shader_src = din.read_string_u32();
                var frag_shader_src = din.read_string_u32();

                if (this.programs[name] == null) {

                    var fragShader = make_shader(gl, gl.FRAGMENT_SHADER, frag_shader_src);
                    var vertexShader = make_shader(gl, gl.VERTEX_SHADER, vertex_shader_src);

                    var program = gl.createProgram();
                    gl.attachShader(program, fragShader);
                    gl.attachShader(program, vertexShader);
                    gl.linkProgram(program);

                    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
                        alert("shader init failed: "+gl.getProgramInfoLog(program));
                    }

                    this.programs[name] = program;
                }

                break;
            }

            case VX_MESSAGE_UNDEFINE_PROGRAM: {
                var name = din.read_id();
                delete this.programs[name];
                console.log("no delprogram implemented, can't delete "+name);
                // XXX
                break;
            }

            case VX_MESSAGE_DEFINE_VERTEX_ATTRIBUTE: {
                var name = din.read_id();
                var type = din.read_u8(); // float=2, int32=3
                var nelements = din.read_u32();
                var ndim = din.read_u8();

                var attr = gl.createBuffer();
                attr.ndim = ndim;

                var buffer = null;

                if (type == 2) {
                    // XXX optimize?
                    var padding = din.read_u8();

                    buffer = din.read_f32_array(nelements);
                    attr.type = gl.FLOAT;

                } else if (type == 3) {

                    buffer = new Int32Array(nelements);
                    for (var i = 0; i < nelements; i++)
                        buffer[i] = din.read_u32();
                    attr.type = gl.INT;

                } else {
                    alert("attr unimplemented");
                }

                if (this.attributes[name] == null) {
                    gl.bindBuffer(gl.ARRAY_BUFFER, attr);
                    gl.bufferData(gl.ARRAY_BUFFER, buffer, gl.STATIC_DRAW);
                    this.attributes[name] = attr;
                }

                break;
            }

            // undefine vertex attribute
            case VX_MESSAGE_UNDEFINE_VERTEX_ATTRIBUTE: {
                var name = din.read_id();

                gl.deleteBuffer(this.attributes[name]);
                delete this.attributes[name];
                break;
            }

            case VX_MESSAGE_DEFINE_INDEX_ARRAY: {
                var name = din.read_id();
                var type = din.read_u8(); // float=2, int32=3
                var nelements = din.read_u32();

                var attr = gl.createBuffer();
                attr.ndim = ndim;

                if (type == 6) { // idx_u16
                    var buffer = new Uint16Array(nelements);
                    for (var i = 0; i < nelements; i++)
                        buffer[i] = din.read_u16();
                    attr.type = gl.SHORT;
                } else {
                    alert("index array unimplemented");
                }

                if (vc.indices[name] == null) {
                    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, attr);
                    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, buffer, gl.STATIC_DRAW);
                    vc.indices[name] = attr;
                }

                break;
            }

            case VX_MESSAGE_UNDEFINE_INDEX_ARRAY: {
                var name = din.read_id();

                gl.deleteBuffer(this.indices[name]);
                delete this.indices[name];
                break;
            }

            case VX_MESSAGE_DEFINE_TEXTURE: {
                var name = din.read_id();
                var width = din.read_u32(); // in pixels
                var height = din.read_u32();
                var stride = din.read_u32();  // in bytes

                // 0: 4 bytes per pixel, R,G,B,A
                // 1: 3 bytes per pixel, R,G,B
                // 2: 1 byte per pixel (gray)
                var pixel_encoding = din.read_u32();

                var flags = din.read_u32();

                var compression = din.read_u32();

                var nbytes = din.read_u32();
                var pixels = din.read_N(nbytes);

                if (compression == 1) {
                    var uc5 = new UC5(pixels);
                    pixels = uc5.uncompress();
                }

                if (this.textures[name] == null) {

                    var tex = gl.createTexture();
                    gl.bindTexture(gl.TEXTURE_2D, tex);

                    if (pixel_encoding == 0)
                        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, stride / 4, height, 0, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
                    else if (pixel_encoding == 1)
                        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, stride / 3, height, 0, gl.RGB, gl.UNSIGNED_BYTE, pixels);
                    else if (pixel_encoding == 2)
                        gl.texImage2D(gl.TEXTURE_2D, 0, gl.LUMINANCE, stride, height, 0, gl.LUMINANCE, gl.UNSIGNED_BYTE, pixels);
                    else
                        alert("unknown image encoding.");

                    if (flags & 1) {
                        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT);
                        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.REPEAT);
                    } else {
                        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
                        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
                    }

                    if (flags & 2)
                        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
                    else
                        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);

                    if (flags & 4)
                        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
                    else
                        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);

                    if (flags & 8) {
                        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR_MIPMAP_NEAREST);
                        gl.generateMipmap(gl.TEXTURE_2D);
                    }

                    gl.bindTexture(gl.TEXTURE_2D, null);

                    this.textures[name] = tex;
                }

                break;
            }

            case VX_MESSAGE_UNDEFINE_TEXTURE: {
                var name = din.read_id();
                gl.deleteTexture(this.textures[name]);
                delete this.textures[name];
                break;
            }

            case VX_MESSAGE_DEFINE_NAMED_MATRIX: {
                var matrixname = din.read_string_u32();
                var M = mat(4, 4);
                for (var i = 0; i < 16; i++)
                    M[i] = din.read_f64();

                // XXX IMPLEMENT
                break;
            }

            case VX_MESSAGE_UNDEFINE_NAMED_MATRIX: {
                var matrixname = din.read_string_u32();
                // XXX IMPLEMENT
                break;
            }

            case VX_MESSAGE_BUFFER_DESTROY: {
                var layername = din.read_string_u32();
                var buffername = din.read_string_u32();

                var vl = this.getLayer(layername);

                vl.buffers = vl.buffers.filter(function(x) {
                    return (x.name != buffername)
                });

                this.notifyBufferLayerDrawOrder();
                this.requestRedraw();

                break;
            }

            case VX_MESSAGE_BUFFER_REDRAW: {
                var layername = din.read_string_u32();
                var buffername = din.read_string_u32();

                var vl = this.getLayer(layername);

                var draworder = din.read_f32();
                var len = din.read_u32();
                var data = din.read_N(len);

                var found_one = false;

                for (var bufferidx = 0; bufferidx < vl.buffers.length; bufferidx++) {
                    if (vl.buffers[bufferidx].name == buffername) {
                        vl.buffers[bufferidx].data = data;
                        if (vl.buffers[bufferidx].draworder != draworder) {
                            vl.buffers[bufferidx].draworder = draworder;
                            this.notifyBufferLayerDrawOrder();
                        }
                        found_one = true;
                        break;
                    }
                }

                if (!found_one) {
                    vl.buffers.push(new VxBuffer(buffername, data));
                    this.notifyBufferLayerDrawOrder();
                }

                this.requestRedraw();

                break;
            }

            case VX_MESSAGE_CANVAS_ECHO: {
                var nonce = din.read_f64();

                var douts = new DataOutput(128);

                douts.write_u32(0x1186712a); // magic
                douts.write_u32(VX_EVENT_CANVAS_ECHO);
                douts.write_f64(nonce);

                // always send echo, don't pass through sendMessage,
                // which does rate limiting.
                this.ws.send(douts.data.subarray(0, douts.datapos));
                break;
            }

            case VX_MESSAGE_CANVAS_SET_SIZE: {
                var width = din.read_u32();
                var height = din.read_u32();

                this.glcanvas.width = width;
                this.glcanvas.height = height;
                this.glcanvas.no_resize = true;
                this.requestRedraw();
                break;
            }

            case VX_MESSAGE_CANVAS_READ_PIXELS: {

                this.draw(); // don't just request a redraw, do it!

                var id = din.read_u64();

                var width = this.glcanvas.width;
                var height = this.glcanvas.height;
                var bytes_per_pixel = 4;
                var buf = new Uint8Array(width * height * bytes_per_pixel);
                var res = gl.readPixels(0, 0, width, height, gl.RGBA, gl.UNSIGNED_BYTE, buf);
                console.log("readpixels id "+id+" "+res);

                var douts = new DataOutput(1024 + buf.length);
                douts.write_u32(0x1186712a); // magic
                douts.write_u32(2000); // READPIXELS
                douts.write_u64(id);
                douts.write_u32(width);
                douts.write_u32(height);
                douts.write_u32(bytes_per_pixel);
                douts.write_array(buf);
                this.ws.send(douts.data.subarray(0, douts.datapos));

                break;
            }

            case VX_MESSAGE_CANVAS_ECHO: {
                var nonce = din.read_f64();

                var douts = new DataOutput(1024);
                douts.write_u32(0x1186712a); // magic
                douts.write_u32(VX_EVENT_ECHO);
                douts.write_f64(nonce);
                this.ws.send(douts.data.subarray(0, douts.datapos));

                break;
            }

            case VX_MESSAGE_CANVAS_SET_TITLE: {
                var title = din.read_string_u32();
                document.title = title;
                break;
            }

            // XXX We should add methods that allow modification of the manipulation point

            // XXX We should add methods that allow modification of camera animation speed

            case VX_MESSAGE_LAYER_ENABLE_CAMERA_CONTROLS: {
                var layername = din.read_string_u32();
                var layer = this.getLayer(layername);
                var mask = din.read_u32();

                for (var handleridx = 0; handleridx < layer.event_handlers.length; handleridx++) {
                    if (layer.event_handlers[handleridx] instanceof DefaultCameraControls)
                        layer.event_handlers[handleridx].cameraControlMask = mask;
                }
                this.requestRedraw();
                break;
            }

            case VX_MESSAGE_LAYER_SET_DRAW_ORDER: {
                var layername = din.read_string_u32();
                var layer = this.getLayer(layername);

                layer.draworder = din.read_f32();
                this.requestRedraw();
                break;
            }

            case VX_MESSAGE_LAYER_SET_BACKGROUND_COLOR: {
                var layername = din.read_string_u32();
                var layer = this.getLayer(layername);

                var rgba = [ din.read_f32(),
                              din.read_f32(),
                              din.read_f32(),
                              din.read_f32() ];

                var dmtime = din.read_f32();

                for (var handleridx = 0; handleridx < layer.event_handlers.length; handleridx++) {
                    if (layer.event_handlers[handleridx] instanceof DefaultCameraControls)
                        layer.event_handlers[handleridx].gotoBackground(rgba, dmtime);
                }

                this.requestRedraw();
                break;
            }

            case VX_MESSAGE_LAYER_SET_POSITION: {
                var layername = din.read_string_u32();
                var layer = this.getLayer(layername);

                var mtime0 = new Date().getTime();

                var vp0 = layer.viewport;
                var vp1 = [ din.read_f32(), din.read_f32(), din.read_f32(), din.read_f32() ];
                var mtime1 = mtime0 + din.read_f32();

                // XXX
                //                vl.layerViewportComputer = new InterpolatingLayerViewportComputer(vp0, mtime0,
//                                                                                  vp1, mtime1);

                this.requestRedraw();

                break;
            }

            case VX_MESSAGE_LAYER_SET_ELU: {
                var layername = din.read_string_u32();
                var layer = this.getLayer(layername);

                var eye1    = [ din.read_f64(), din.read_f64(), din.read_f64() ];
                var lookat1 = [ din.read_f64(), din.read_f64(), din.read_f64() ];
                var up1     = [ din.read_f64(), din.read_f64(), din.read_f64() ];
                var dmtime  = din.read_f32();

                for (var handleridx = 0; handleridx < layer.event_handlers.length; handleridx++) {
                    if (layer.event_handlers[handleridx] instanceof DefaultCameraControls)
                        layer.event_handlers[handleridx].gotoELU(eye1, lookat1, up1, dmtime);
                }

                this.requestRedraw();
                break;
            }

            case VX_MESSAGE_LAYER_SET_CAMERA_MODE: {
                var layername = din.read_string_u32();
                var layer = this.getLayer(layername);
                var mode = din.read_string_u32();

                for (var handleridx = 0; handleridx < layer.event_handlers.length; handleridx++) {
                    if (layer.event_handlers[handleridx] instanceof DefaultCameraControls)
                        layer.event_handlers[handleridx].interfaceMode = mode;
                }

                this.requestRedraw();
                break;
            }

            default: {
                console.log("unknown opcode at "+din.datapos);
                break;
            }
        }// switch (opcode)
    }
}

function VxRemoteEventHandler()
{
    this.eventorder = 100; // last.
}

VxRemoteEventHandler.prototype.makeBasicEventResponse = function(layer, e, code)
{
    // send event back to the client
    var douts = new DataOutput(1024);
    douts.write_u32(0x1186712a); // magic
    douts.write_u32(code);
    douts.write_u64(e.timeStamp * 1000 );    // utime on client
    douts.write_string_u32(layer.name);

    for (var i = 0; i < 4; i++)
        douts.write_f32(layer.viewport[i]);

    for (var i = 0; i < 16; i++)
        douts.write_f64(layer.P[i]);

    for (var i = 0; i < 16; i++)
        douts.write_f64(layer.V[i]);

    var flags = 0;
    if (e.altKey)
        flags |= 1;
    if (e.ctrlKey)
        flags |= 2;
    if (e.shiftKey)
        flags |= 4;
    douts.write_u32(flags);

    return douts;
}

VxRemoteEventHandler.prototype.onEvent = function(layer, type, e)
{
    var vc = layer.vc;
    var douts = null;

    switch (type)
    {
        case "onkeydown": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_KEY_DOWN);
            douts.write_u32(e.keyCode);
            break;
        }

        case "onkeypress": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_KEY_PRESSED);
            var keyCode = (typeof e.which == "number") ? e.which : e.keyCode;
            // For inter-operability between Chrome and Firefox
            douts.write_u32(keyCode);
            // douts.write_u32(e.keyCode);
            break;
        }

        case "onkeyup": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_KEY_UP);
            douts.write_u32(e.keyCode);
            break;
        }

        case "onmousewheel": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_MOUSE_WHEEL);
            douts.write_f32(e.wheelDelta);
            break;
        }

        case "onmousedown": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_MOUSE_DOWN);
            douts.write_f32(e.xy[0]);
            douts.write_f32(e.xy[1]);
            douts.write_u8(e.button);
            break;
        }

        case "onmousemove": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_MOUSE_MOVED);
            douts.write_f32(e.xy[0]);
            douts.write_f32(e.xy[1]);
            break;
        }

        case "onmouseclick": {
            // logic to filter drags omitted.
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_MOUSE_CLICKED);
            douts.write_f32(e.xy[0]);
            douts.write_f32(e.xy[1]);
            break;
        }

        case "onmouseup": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_MOUSE_UP);
            douts.write_f32(e.xy[0]);
            douts.write_f32(e.xy[1]);
            douts.write_u8(e.button);
            break;
        }

        case "ontouchstart": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_TOUCH_START);
            douts.write_f32(e.xy[0]);
            douts.write_f32(e.xy[1]);
            douts.write_u32(e.touches.length);
            douts.write_u32(e.changedTouches[0].identifier);
            break;
        }

        case "ontouchmove": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_TOUCH_MOVE);
            douts.write_f32(e.xy[0]);
            douts.write_f32(e.xy[1]);
            douts.write_u32(e.touches.length);
            douts.write_u32(e.changedTouches[0].identifier);
            break;
        }

        case "ontouchend": {
            douts = this.makeBasicEventResponse(layer, e, VX_EVENT_TOUCH_END);
            douts.write_f32(e.xy[0]);
            douts.write_f32(e.xy[1]);
            douts.write_u32(e.touches.length);
            douts.write_u32(e.changedTouches[0].identifier);
            break;
        }

        default: {
            console.log("unknown event type "+type);
            break;
        }
    }

    if (douts) {
        vc.sendMessage(type, douts);
        return true;
    }

    return false;
}
