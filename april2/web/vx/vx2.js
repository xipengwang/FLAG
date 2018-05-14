"use strict";

function VxCanvas(canvasid)
{
    this.canvasid = canvasid;
    this.glcanvas = document.getElementById(canvasid);

    this.gl = this.glcanvas.getContext("webgl", {preserveDrawingBuffer: true });
    if (this.gl == null)
        this.gl = this.glcanvas.getContext("experimental-webgl");

    if (this.gl == null)
        this.gl = this.glcanvas.getContext("experimental-webgl");

    //    this.gl = WebGLDebugUtils.makeDebugContext(this.gl, throwOnGLError, logGLCall);

    let gl = this.gl;

    this.on_draw_callbacks = [];
    this.on_layer_callbacks = [];

    this.glcanvas.addEventListener('contextmenu', function(e) { e.preventDefault(); });
    this.glcanvas.oncontextmenu = function() { return false; };

    this.glcanvas.focus();

    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LESS);

    gl.enable(gl.BLEND);
    gl.enable(gl.SCISSOR_TEST);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

    // .name, .buffers, .draworder, .renderinfo, .projinfo, .event_handlers
    this.layers = [];

    this.glcanvas.onmousewheel = (e) => { this.onmousewheel(e); };

    this.glcanvas.addEventListener('DOMMouseScroll',
                                   function(e) {
                                       e.wheelDelta = Math.max(-1, Math.min(1, (e.wheelDelta || -e.detail))); //FF hack
                                       _vw.onmousewheel(e);
                                   });
    this.glcanvas.onmousedown  = (e) => { return this.onmousedown(e); };
    this.glcanvas.onmouseup    = (e) => { return this.onmouseup(e); };
    this.glcanvas.onmousemove  = (e) => { return this.onmousemove(e); };
    this.glcanvas.onclick      = (e) => { return this.onmouseclick(e); };

    this.glcanvas.ontouchstart = (e) => { return this.ontouchstart(e); };
    this.glcanvas.ontouchmove  = (e) => { return this.ontouchmove(e); };
    this.glcanvas.ontouchend   = (e) => { return this.ontouchend(e); };

    this.glcanvas.onkeypress   = (e) => { return this.onkeypress(e); };
    this.glcanvas.onkeydown    = (e) => { return this.onkeydown(e); };
    this.glcanvas.onkeyup      = (e) => { return this.onkeyup(e); };

    let oldfn = document.onmouseup;
    document.onmouseup = (e) => { this.onmouseup(e); if (oldfn) return oldfn(e); };

    this.glcanvas.force_redraw = false;

    window.setInterval(() => {
        if (this.dirty) {
            this.dirty = 0;

           if (this.glcanvas.force_redraw)
               this.draw();
           else
               requestAnimationFrame(() => { this.draw(); });
        }
    }, 30);
};

VxCanvas.prototype.getLayer = function(name)
{
    for (let layeridx = this.layers.length - 1; layeridx >= 0; layeridx--) {
        let layer = this.layers[layeridx];
        if (layer.name == name)
            return layer;
    }

    let layer = { };
    layer.name = name;
    layer.buffers = [ ];
    layer.draworder = 0;
    layer.event_handlers = [];
    this.layers.push(layer);

    return layer;
}

VxCanvas.prototype.computeEventCoordinates = function(clientX, clientY)
{
    let devicePixelRatio = window.devicePixelRatio || 1;

    let xy = [ (clientX - this.glcanvas.offsetLeft) * devicePixelRatio,
               (clientY - this.glcanvas.offsetTop) * devicePixelRatio ];

    return xy;
};

VxCanvas.prototype.dispatch = function(e, handlername)
{
    let xy = this.computeEventCoordinates(e.clientX, e.clientY);

    // process from "topmost" to "bottommost"
    for (let layeridx = this.layers.length - 1; layeridx >= 0; layeridx--) {

        let layer = this.layers[layeridx];
        let viewport = layer.renderinfo.viewport;

        if (xy[0] >= viewport[0] && xy[0] < viewport[0] + viewport[2] &&
            (ch - xy[1]) >= viewport[1] && (ch - xy[1]) < viewport[1] + viewport[3]) {

            for (let handleridx = 0; handleridx < layer.event_handlers.length; handleridx++) {
                if (layer.event_handlers[handleridx][handlername](e)) {
                    this.mouseDownLayer = layer;
                    return layer;
                }
            }
        }
    }

    return null;
};

VxCanvas.prototype.onmousedown = function(e)
{
    let layer = this.dispatch(e, "onmousedown");
    if (layer)
        this.mouseDownLayer = layer;

    return layer != null;
};

VxCanvas.prototype.draw = function()
{
    stable_sort(this.layers, function(a, b) { return a.draworder - b.draworder });

    let mtime = new Date().getTime();

    // from bottom-most to top-most
    for (let layeridx = 0; layeridx < this.layers.length; layeridx++) {
        let layer = this.layers[layeridx];

        stable_sort(layer.buffers, function(a, b) { return a.draworder - b.draworder });

        this.renderinfo = new Object();

        // .viewport, .done
        renderinfo.viewport_info = this.computeViewport(mtime, this.canvas.getWidth(), this.canvas.getHeight());

        // .eye, .lookat, .up, .done
        renderinfo.elu_info = this.computeELU(mtime);

        // .fovy_degrees, .zclip_near, .zclip_far, .P, .perspectiveness, .done
        renderinfo.proj_info = this.computeProj(mtime, renderinfo.viewport[2], renderinfo.viewport[3]);

        var stack = { M: new MatrixStack(this.renderinfo.projinfo.P),
                      P: new MatrixStack(this.renderinfo.elu.V),
                      V: new MatrixStack(mat_identity(4)),
                      depth_test: new EnableStack(1)
                    };

        var viewport = this.renderinfo.viewport;

        gl.scissor(viewport[0], viewport[1], viewport[2], viewport[3]);
        gl.viewport(viewport[0], viewport[1], viewport[2], viewport[3]);

        gl.clearColor(this.background_rgba[0], this.background_rgba[1], this.background_rgba[2], this.background_rgba[3]);

        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

        for (var i = 0; i < this.buffers.length; i++) {
            if (this.visible && this.buffers[i].visible)
                this.buffers[i].draw(stack);
        }

        if (!this.renderinfo.elu.done || !this.renderinfo.projinfo.done || !this.renderinfo.viewport.done)
            this.canvas.onDirty();
    }


}
