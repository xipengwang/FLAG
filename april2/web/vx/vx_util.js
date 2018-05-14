"use strict";

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


function elu_to_matrix(eye, lookat, up)
{
    up = vec_normalize(up);
    var f = vec_normalize(vec_subtract(lookat, eye));

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
