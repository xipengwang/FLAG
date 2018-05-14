
/////////////////////////////////////////////////////////
function VwRobot()
{
    VwObject.call(this);
}

VwRobot.prototype = Object.create(VwObject.prototype);

VwRobot.prototype.draw = function(canvas, layer, stack)
{
    var gl = canvas.gl;

    if (this.vertexBuffer == null) {
        var vertices = [  0.0, 1.0, 0.0,
                          -1.0, -1.0, 0.0,
                          1.0, -1.0, 0.0 ];

        var buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(vertices), gl.STATIC_DRAW);
        buf.ndim = 3;
        buf.size = vertices.length / buf.ndim;

        VwRobot.prototype.vertexBuffer = buf;

        var frag_shader_src =
            "precision mediump float; \n" +
            "void main(void) {\n" +
            "  gl_FragColor = vec4(0.5, 1.0, 1.0, 1.0); \n" +
            "}\n";

        var vertex_shader_src =
            "attribute vec3 position; \n"+
            "uniform mat4 P;\n" +
            "uniform mat4 V;\n" +
            "uniform mat4 M;\n" +
            "void main(void) {\n" +
            "  gl_Position = P * V * M * vec4(position, 1.0);\n"+
            "}";

        var fragShader = make_shader(gl, gl.FRAGMENT_SHADER, frag_shader_src);
        var vertexShader = make_shader(gl, gl.VERTEX_SHADER, vertex_shader_src);

        var program = gl.createProgram();
        gl.attachShader(program, fragShader);
        gl.attachShader(program, vertexShader);
        gl.linkProgram(program);

        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            alert("shader init failed");
        }

        program.positionAttribute = gl.getAttribLocation(program, "position");
        program.Puniform = gl.getUniformLocation(program, "P");
        program.Vuniform = gl.getUniformLocation(program, "V");
        program.Muniform = gl.getUniformLocation(program, "M");

        VwRobot.prototype.program = program;
    }

    gl.useProgram(this.program);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vertexBuffer);

    gl.enableVertexAttribArray(this.program.positionAttribute);
    gl.vertexAttribPointer(this.program.positionAttribute, this.vertexBuffer.ndim, gl.FLOAT, false, 0, 0);

    gl.uniformMatrix4fv(this.program.Puniform, false, mat_transpose(stack.P.get()));
    gl.uniformMatrix4fv(this.program.Vuniform, false, mat_transpose(stack.V.get()));
    gl.uniformMatrix4fv(this.program.Muniform, false, mat_transpose(stack.M.get()));

    gl.drawArrays(gl.TRIANGLES, 0, this.vertexBuffer.size);
};


/////////////////////////////////////////////////////////
// vertices is of type Float32Array
function VwSolidPoints(vertices)
{
    VwObject.call(this);
    this.vertices = vertices;
}

VwSolidPoints.prototype = Object.create(VwObject.prototype);

VwSolidPoints.prototype.draw = function(canvas, layer, stack)
{
    var gl = canvas.gl;

    if (this.vertexBuffer == null) {

        var buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, this.vertices, gl.STATIC_DRAW);
        buf.ndim = 3;
        buf.size = this.vertices.length / buf.ndim;
        this.vertexBuffer = buf;
        this.vertices = null; // allow GC
    }

    if (this.program == null) {
        var frag_shader_src =
            "precision mediump float; \n" +
            "void main(void) {\n" +
            "  gl_FragColor = vec4(0.5, 1.0, 1.0, 1.0); \n" +
            "}\n";

        var vertex_shader_src =
            "attribute vec3 position; \n"+
            "uniform mat4 P;\n" +
            "uniform mat4 V;\n" +
            "uniform mat4 M;\n" +
            "void main(void) {\n" +
            "  gl_Position = P * V * M * vec4(position, 1.0);\n"+
            "}";

        var fragShader = make_shader(gl, gl.FRAGMENT_SHADER, frag_shader_src);
        var vertexShader = make_shader(gl, gl.VERTEX_SHADER, vertex_shader_src);

        var program = gl.createProgram();
        gl.attachShader(program, fragShader);
        gl.attachShader(program, vertexShader);
        gl.linkProgram(program);

        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            alert("shader init failed");
        }

        program.positionAttribute = gl.getAttribLocation(program, "position");
        program.Puniform = gl.getUniformLocation(program, "P");
        program.Vuniform = gl.getUniformLocation(program, "V");
        program.Muniform = gl.getUniformLocation(program, "M");

        VwSolidPoints.prototype.program = program;
    }

    gl.useProgram(this.program);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vertexBuffer);

    gl.enableVertexAttribArray(this.program.positionAttribute);
    gl.vertexAttribPointer(this.program.positionAttribute, this.vertexBuffer.ndim, gl.FLOAT, false, 0, 0);

    gl.uniformMatrix4fv(this.program.Puniform, false, mat_transpose(stack.P.get()));
    gl.uniformMatrix4fv(this.program.Vuniform, false, mat_transpose(stack.V.get()));
    gl.uniformMatrix4fv(this.program.Muniform, false, mat_transpose(stack.M.get()));

    gl.drawArrays(gl.POINTS, 0, this.vertexBuffer.size);
};


/////////////////////////////////////////////////////////
// vertices is of type Float32Array
function VwZColoredPoints(vertices)
{
    VwObject.call(this);
    this.vertices = vertices;
}

VwZColoredPoints.prototype = Object.create(VwObject.prototype);

VwZColoredPoints.prototype.draw = function(canvas, layer, stack)
{
    var gl = canvas.gl;

    if (this.vertexBuffer == null) {

        var buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, this.vertices, gl.STATIC_DRAW);
        buf.ndim = 3;
        buf.size = this.vertices.length / buf.ndim;
        this.vertexBuffer = buf;
        this.vertices = null; // allow GC
    }

    if (this.program == null) {
        var frag_shader_src =
            "#ifdef GL_ES\n"+
            "precision mediump float; \n" +
            "#endif\n"+
            "varying vec4 c;\n" +
            "void main(void) {\n" +
            "  gl_FragColor = c; \n" +
            "}\n";

        var vertex_shader_src =
            "precision mediump float; \n"+
            "uniform mat4 P;\n" +
            "uniform mat4 V;\n" +
            "uniform mat4 M;\n" +

            "attribute vec3 position; \n"+
            "varying vec4 c;\n" +

            "void main(void) {\n" +
            "  gl_Position = P * V * M * vec4(position, 1.0);\n"+
            "  float f = (position[2] + 2.0) / 10.0;\n" +
            "  float g = (position[2] + 1.0) / 5.0;\n"+
            "  float h = (position[2] + .1) / 0.25;\n"+
            "  c = vec4(f, g, h, 1.0);\n"+
            "  gl_PointSize = 2.0;\n" +
            "}";

        var fragShader = make_shader(gl, gl.FRAGMENT_SHADER, frag_shader_src);
        var vertexShader = make_shader(gl, gl.VERTEX_SHADER, vertex_shader_src);

        var program = gl.createProgram();
        gl.attachShader(program, fragShader);
        gl.attachShader(program, vertexShader);
        gl.linkProgram(program);

        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            alert("shader init failed");
        }

        program.positionAttribute = gl.getAttribLocation(program, "position");
        program.Puniform = gl.getUniformLocation(program, "P");
        program.Vuniform = gl.getUniformLocation(program, "V");
        program.Muniform = gl.getUniformLocation(program, "M");

        VwZColoredPoints.prototype.program = program;
    }

    gl.useProgram(this.program);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vertexBuffer);

    gl.enableVertexAttribArray(this.program.positionAttribute);
    gl.vertexAttribPointer(this.program.positionAttribute, this.vertexBuffer.ndim, gl.FLOAT, false, 0, 0);

    gl.uniformMatrix4fv(this.program.Puniform, false, mat_transpose(stack.P.get()));
    gl.uniformMatrix4fv(this.program.Vuniform, false, mat_transpose(stack.V.get()));
    gl.uniformMatrix4fv(this.program.Muniform, false, mat_transpose(stack.M.get()));

    gl.drawArrays(gl.POINTS, 0, this.vertexBuffer.size);
};

/////////////////////////////////////////////////////////
// vertices is of type Float32Array
function VwZColoredIntensityPoints(xyzi)
{
    VwObject.call(this);
    this.xyzi = xyzi;
}

VwZColoredIntensityPoints.prototype = Object.create(VwObject.prototype);

VwZColoredIntensityPoints.prototype.draw = function(canvas, layer, stack)
{
    var gl = canvas.gl;

    if (this.vertexBuffer == null) {

        var buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, this.xyzi, gl.STATIC_DRAW);
        buf.ndim = 4;
        buf.size = this.xyzi.length / buf.ndim;
        this.vertexBuffer = buf;
        this.xyzi = null; // allow GC
    }

    if (this.program == null) {
        var frag_shader_src =
            "#ifdef GL_ES\n"+
            "precision mediump float; \n" +
            "#endif\n"+
            "varying vec4 c;\n" +
            "void main(void) {\n" +
//            " gl_FragColor = vec4(1.0, 1.0, 0.0, 1.0);\n" +
            "  gl_FragColor = c; \n" +
            "}\n";

        var vertex_shader_src =
            "precision mediump float; \n"+
            "uniform mat4 P;\n" +
            "uniform mat4 V;\n" +
            "uniform mat4 M;\n" +

            "attribute vec4 position; \n"+
            "varying vec4 c;\n" +

            "void main(void) {\n" +
            "  gl_Position = P * V * M * vec4(position.x, position.y, position.z, 1.0);\n"+
            "  float delta = 0.15; \n"+
            "  float md = .05;\n"+
//            "  float f = mod(position[2], md)/md, g = position[3], h = f;\n" +
            "  float f = (position[2] + delta) / delta, g = position[3], h = 1.0 - (position[2]/delta);\n"+
//            "  float f = (position[2] + delta) / delta;\n" +
//            "  float g = position[3];\n" +
//            "  float h = 1.0 - (position[2] + 0.0) / delta;\n"+

            "  c = vec4(f, g, h, 1.0);\n"+
            "  gl_PointSize = 1.0;\n" +
            "}";

        var fragShader = make_shader(gl, gl.FRAGMENT_SHADER, frag_shader_src);
        var vertexShader = make_shader(gl, gl.VERTEX_SHADER, vertex_shader_src);

        var program = gl.createProgram();
        gl.attachShader(program, fragShader);
        gl.attachShader(program, vertexShader);
        gl.linkProgram(program);

        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            alert("shader init failed");
        }

        program.positionAttribute = gl.getAttribLocation(program, "position");
        program.Puniform = gl.getUniformLocation(program, "P");
        program.Vuniform = gl.getUniformLocation(program, "V");
        program.Muniform = gl.getUniformLocation(program, "M");

        VwZColoredIntensityPoints.prototype.program = program;
    }

    gl.useProgram(this.program);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vertexBuffer);

    gl.enableVertexAttribArray(this.program.positionAttribute);
    gl.vertexAttribPointer(this.program.positionAttribute, this.vertexBuffer.ndim, gl.FLOAT, false, 0, 0);

    gl.uniformMatrix4fv(this.program.Puniform, false, mat_transpose(stack.P.get()));
    gl.uniformMatrix4fv(this.program.Vuniform, false, mat_transpose(stack.V.get()));
    gl.uniformMatrix4fv(this.program.Muniform, false, mat_transpose(stack.M.get()));

    gl.drawArrays(gl.POINTS, 0, this.vertexBuffer.size);
};

/////////////////////////////////////////////////////////
// vertices is of type Float32Array
function VwLineStrip(xyz, r, g, b, a)
{
    VwObject.call(this);
    this.xyz = xyz;

    this.frag_shader_src =
        "#ifdef GL_ES\n"+
        "precision mediump float; \n" +
        "#endif\n"+
        "varying vec4 c;\n" +
        "void main(void) {\n" +
        " gl_FragColor = vec4("+r+", "+g+", "+b+", "+a+");\n" +
        "}\n";

    this.vertex_shader_src =
        "precision mediump float; \n"+
        "uniform mat4 P;\n" +
        "uniform mat4 V;\n" +
        "uniform mat4 M;\n" +
        "attribute vec4 position; \n"+
        "varying vec4 c;\n" +
        "void main(void) {\n" +
        "  gl_Position = P * V * M * vec4(position.x, position.y, position.z, 1.0);\n"+
        "}";
}

VwLineStrip.prototype = Object.create(VwObject.prototype);

VwLineStrip.prototype.draw = function(canvas, layer, stack)
{
    var gl = canvas.gl;

    if (this.vertexBuffer == null) {

        var buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, this.xyz, gl.STATIC_DRAW);
        buf.ndim = 3;
        buf.size = this.xyz.length / buf.ndim;
        this.vertexBuffer = buf;
        this.xyz = null; // allow GC
    }

    if (this.program == null) {

        var fragShader = make_shader(gl, gl.FRAGMENT_SHADER, this.frag_shader_src);
        var vertexShader = make_shader(gl, gl.VERTEX_SHADER, this.vertex_shader_src);

        var program = gl.createProgram();
        gl.attachShader(program, fragShader);
        gl.attachShader(program, vertexShader);
        gl.linkProgram(program);

        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            alert("shader init failed");
        }

        program.positionAttribute = gl.getAttribLocation(program, "position");
        program.Puniform = gl.getUniformLocation(program, "P");
        program.Vuniform = gl.getUniformLocation(program, "V");
        program.Muniform = gl.getUniformLocation(program, "M");

        this.program = program;
    }

    gl.useProgram(this.program);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vertexBuffer);

    gl.enableVertexAttribArray(this.program.positionAttribute);
    gl.vertexAttribPointer(this.program.positionAttribute, this.vertexBuffer.ndim, gl.FLOAT, false, 0, 0);

    gl.uniformMatrix4fv(this.program.Puniform, false, mat_transpose(stack.P.get()));
    gl.uniformMatrix4fv(this.program.Vuniform, false, mat_transpose(stack.V.get()));
    gl.uniformMatrix4fv(this.program.Muniform, false, mat_transpose(stack.M.get()));

    gl.drawArrays(gl.LINE_STRIP, 0, this.vertexBuffer.size);
};

///////////////////////////////////////////////////////////////////////////
function VwTextureTest()
{
    VwObject.call(this);

    this.frag_shader_src =
        "#ifdef GL_ES\n"+
        "precision mediump float; \n" +
        "#endif\n"+
        "uniform sampler2D texture;\n" +
        "varying vec2 texCoord;\n" +
        "void main(void) {\n" +
        " gl_FragColor = texture2D(texture, texCoord);\n" +
        "}\n";

    this.vertex_shader_src =
        "precision mediump float; \n"+
        "uniform mat4 P;\n" +
        "uniform mat4 V;\n" +
        "uniform mat4 M;\n" +
        "attribute vec3 position; \n"+
        "attribute vec2 vertexTexCoord;\n" +
        "varying vec2 texCoord;\n" +
        "void main(void) {\n" +
        "  gl_Position = P * V * M * vec4(position.x, position.y, position.z, 1.0);\n"+
        "  texCoord = vertexTexCoord;\n" +
        "}";
}

VwTextureTest.prototype = Object.create(VwObject.prototype);

VwTextureTest.prototype.draw = function(canvas, layer, stack)
{
    var gl = canvas.gl;

    var w = 2;
    var h = 2;
    var s = 4;

    if (this.xyBuffer == null) {
        var positionData = [ 0.0, 0.0,
                       1.0, 0.0,
                       0.0, 1.0,
                       1.0, 1.0 ];

        var texData = [ 0.0, 0.0,
                        w,   0.0,
                        0.0, h,
                        w,   h ];

        this.positionBuffer = gl.createBuffer();
        this.positionBuffer.ndim = 2;
        this.positionBuffer.size = positionData.length / this.positionBuffer.ndim;
        gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positionData), gl.STATIC_DRAW);

        this.texBuffer = gl.createBuffer();
        this.texBuffer.ndim = 2;
        this.texBuffer.size = texData.length / this.texBuffer.ndim;
        gl.bindBuffer(gl.ARRAY_BUFFER, this.texBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(texData), gl.STATIC_DRAW);
    }

    if (this.program == null) {

        var fragShader = make_shader(gl, gl.FRAGMENT_SHADER, this.frag_shader_src);
        var vertexShader = make_shader(gl, gl.VERTEX_SHADER, this.vertex_shader_src);

        var program = gl.createProgram();
        gl.attachShader(program, fragShader);
        gl.attachShader(program, vertexShader);
        gl.linkProgram(program);

        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            alert("program link failed: "+gl.getProgramInfoLog(program));
        }

        program.positionAttribute = gl.getAttribLocation(program, "position");
        program.Puniform = gl.getUniformLocation(program, "P");
        program.Vuniform = gl.getUniformLocation(program, "V");
        program.Muniform = gl.getUniformLocation(program, "M");

        program.vertexTexCoordAttribute = gl.getAttribLocation(program, "vertexTexCoord");
        program.textureUniform = gl.getUniformLocation(program, "texture");

        this.program = program;
    }

    if (this.texture == null) {
        var data = new Uint8Array(w*h*s);
        data[0*s+0] = 0xff;
        data[0*s+1] = 0x00;
        data[0*s+2] = 0xff;
        data[0*s+3] = 0xff;

        data[1*s+0] = 0xff;
        data[1*s+1] = 0xff;
        data[1*s+2] = 0x00;
        data[1*s+3] = 0xff;

        data[2*s+0] = 0xff;
        data[2*s+1] = 0x00;
        data[2*s+2] = 0x00;
        data[2*s+3] = 0xff;

        data[3*s+0] = 0xff;
        data[3*s+1] = 0xff;
        data[3*s+2] = 0xff;
        data[3*s+3] = 0xff;

        this.texture = gl.createTexture();
        gl.bindTexture(gl.TEXTURE_2D, this.texture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA,
                      w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, data);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
        gl.bindTexture(gl.TEXTURE_2D, null);
    }

    gl.useProgram(this.program);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
    gl.enableVertexAttribArray(this.program.positionAttribute);
    gl.vertexAttribPointer(this.program.positionAttribute, this.positionBuffer.ndim, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.texBuffer);
    gl.enableVertexAttribArray(this.program.vertexTexCoordAttribute);
    gl.vertexAttribPointer(this.program.vertexTexCoordAttribute, this.texBuffer.ndim, gl.FLOAT, false, 0, 0);

    gl.uniformMatrix4fv(this.program.Puniform, false, mat_transpose(stack.P.get()));
    gl.uniformMatrix4fv(this.program.Vuniform, false, mat_transpose(stack.V.get()));
    gl.uniformMatrix4fv(this.program.Muniform, false, mat_transpose(stack.M.get()));

    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.uniform1i(this.textureUniform, 0);

    gl.drawArrays(gl.TRIANGLE_STRIP, 0, this.positionBuffer.size);
};
