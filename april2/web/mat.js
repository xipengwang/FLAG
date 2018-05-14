/* a mat is an array, row-major ordered, with additional integer attributes nrows and ncols. */

function assert(cond)
{
    console.assert(cond, "no message");
}

function mat_create(nrows, ncols, data)
{
    var M;

    if (data != null) {
        M = data.slice();
    } else {
        M = new Array(nrows * ncols);
        for (var i = 0; i < M.length; i++)
            M[i] = 0;
    }

    M.nrows = nrows;
    M.ncols = ncols;

    return M;
}

function mat_copy(M)
{
    var X = mat_create(M.nrows, M.ncols);
    for (var i = 0; i < X.length; i++)
        X[i] = M[i];

    return X;
}

function mat_identity(n)
{
    var M = mat_create(n, n);
    for (var i = 0; i < M.nrows; i++)
        M[i*M.ncols + i] = 1;
    return M;
}

function mat_scale(s)
{
    return mat_create(4, 4, [ s, 0, 0, 0,
                              0, s, 0, 0,
                              0, 0, s, 0,
                              0, 0, 0, 1.0 ]);
}

function mat_translate(tx, ty, tz)
{
    return mat_create(4, 4, [ 1.0, 0, 0, tx,
                              0, 1.0, 0, ty,
                              0, 0, 1.0, tz,
                              0, 0, 0, 1.0 ]);
}

function mat_rotateX(rad)
{
    var c = Math.cos(rad), s = Math.sin(rad);

    return mat_create(4, 4, [ 1, 0, 0, 0,
                              0, c, -s, 0,
                              0, s, c, 0,
                              0, 0, 0, 1.0 ]);
}

function mat_rotateY(rad)
{
    var c = Math.cos(rad), s = Math.sin(rad);

    return mat_create(4, 4, [ c, 0, s, 0,
                              0, 1, 0, 0,
                              -s, 0, c, 0,
                              0, 0, 0, 1.0 ]);
}

function mat_rotateZ(rad)
{
    var c = Math.cos(rad), s = Math.sin(rad);

    return mat_create(4, 4, [ c, -s, 0, 0,
                              s, c, 0, 0,
                              0, 0, 1.0, 0,
                              0, 0, 0, 1.0 ]);
}

function mat_transpose(M)
{
    var X = mat_create(M.ncols, M.nrows);

    for (var i = 0; i < M.nrows; i++)
        for (var j = 0; j < M.ncols; j++)
            X[j*X.nrows + i] = M[i*M.nrows + j];

    return X;
}

function mat_multiply(A, B)
{
    assert(A.ncols == B.nrows);
    var X = mat_create(A.nrows, B.ncols);

    for (var i = 0; i < A.nrows; i++) {
        for (var j = 0; j < A.ncols; j++) {
            var acc = 0;
            for (var k = 0; k < B.nrows; k++) {
                acc += A[A.ncols*i + k]*B[B.ncols*k +j];
            }
            X[X.ncols*i + j] = acc;
        }
    }

    return X;
}

function mat_inverse(X)
{
    if (X.nrows == 2) {
        assert(X.ncols == 2);

        var a = X[0], b = X[1], c = X[2], d = X[3];

        var det = 1.0 / (a*d - b*c);
        return mat_create(2, 2, [ det*d, -det*b,
                                  -det*c, det*a ]);
    }

    if (X.nrows == 4) {
        assert(X.ncols == 4);
        var m00 = X[0],  m01 = X[1],  m02 = X[2],  m03 = X[3];
        var m10 = X[4],  m11 = X[5],  m12 = X[6],  m13 = X[7];
        var m20 = X[8],  m21 = X[9],  m22 = X[10], m23 = X[11];
        var m30 = X[12], m31 = X[13], m32 = X[14], m33 = X[15];

        var det = m00 * m11 * m22 * m33 - m00 * m11 * m23 * m32 -
            m00 * m21 * m12 * m33 + m00 * m21 * m13 * m32 + m00 * m31 * m12 * m23 -
            m00 * m31 * m13 * m22 - m10 * m01 * m22 * m33 +
            m10 * m01 * m23 * m32 + m10 * m21 * m02 * m33 -
            m10 * m21 * m03 * m32 - m10 * m31 * m02 * m23 +
            m10 * m31 * m03 * m22 + m20 * m01 * m12 * m33 -
            m20 * m01 * m13 * m32 - m20 * m11 * m02 * m33 +
            m20 * m11 * m03 * m32 + m20 * m31 * m02 * m13 -
            m20 * m31 * m03 * m12 - m30 * m01 * m12 * m23 +
            m30 * m01 * m13 * m22 + m30 * m11 * m02 * m23 -
            m30 * m11 * m03 * m22 - m30 * m21 * m02 * m13 +
            m30 * m21 * m03 * m12;

        if (det == 0)
            return null;

        var Y = mat_create(4,4);

        Y[0]  =   m11 * m22 * m33 - m11 * m23 * m32 - m21 * m12 * m33 + m21 * m13 * m32 + m31 * m12 * m23 - m31 * m13 * m22;
        Y[4]  = - m10 * m22 * m33 + m10 * m23 * m32 + m20 * m12 * m33 - m20 * m13 * m32 - m30 * m12 * m23 + m30 * m13 * m22;
        Y[8]  =   m10 * m21 * m33 - m10 * m23 * m31 - m20 * m11 * m33 + m20 * m13 * m31 + m30 * m11 * m23 - m30 * m13 * m21;
        Y[12]  = - m10 * m21 * m32 + m10 * m22 * m31 + m20 * m11 * m32 - m20 * m12 * m31 - m30 * m11 * m22 + m30 * m12 * m21;
        Y[1]  = - m01 * m22 * m33 + m01 * m23 * m32 + m21 * m02 * m33 - m21 * m03 * m32 - m31 * m02 * m23 + m31 * m03 * m22;
        Y[5]  =   m00 * m22 * m33 - m00 * m23 * m32 - m20 * m02 * m33 + m20 * m03 * m32 + m30 * m02 * m23 - m30 * m03 * m22;
        Y[9]  = - m00 * m21 * m33 + m00 * m23 * m31 + m20 * m01 * m33 - m20 * m03 * m31 - m30 * m01 * m23 + m30 * m03 * m21;
        Y[13]  =   m00 * m21 * m32 - m00 * m22 * m31 - m20 * m01 * m32 + m20 * m02 * m31 + m30 * m01 * m22 - m30 * m02 * m21;
        Y[2]  =   m01 * m12 * m33 - m01 * m13 * m32 - m11 * m02 * m33 + m11 * m03 * m32 + m31 * m02 * m13 - m31 * m03 * m12;
        Y[6]  = - m00 * m12 * m33 + m00 * m13 * m32 + m10 * m02 * m33 - m10 * m03 * m32 - m30 * m02 * m13 + m30 * m03 * m12;
        Y[10] =   m00 * m11 * m33 - m00 * m13 * m31 - m10 * m01 * m33 + m10 * m03 * m31 + m30 * m01 * m13 - m30 * m03 * m11;
        Y[14] = - m00 * m11 * m32 + m00 * m12 * m31 + m10 * m01 * m32 - m10 * m02 * m31 - m30 * m01 * m12 + m30 * m02 * m11;
        Y[3] = - m01 * m12 * m23 + m01 * m13 * m22 + m11 * m02 * m23 - m11 * m03 * m22 - m21 * m02 * m13 + m21 * m03 * m12;
        Y[7] =   m00 * m12 * m23 - m00 * m13 * m22 - m10 * m02 * m23 + m10 * m03 * m22 + m20 * m02 * m13 - m20 * m03 * m12;
        Y[11] = - m00 * m11 * m23 + m00 * m13 * m21 + m10 * m01 * m23 - m10 * m03 * m21 - m20 * m01 * m13 + m20 * m03 * m11;
        Y[15] =   m00 * m11 * m22 - m00 * m12 * m21 - m10 * m01 * m22 + m10 * m02 * m21 + m20 * m01 * m12 - m20 * m02 * m11;

        for (var i = 0; i < 16; i++)
            Y[i] /= det;

        return Y;
    }

    assert(0);
}

function vec_normalize(V)
{
    V = V.slice();

    var N = V.length;

    var acc = 0;
    for (var i = 0; i < N; i++) {
        acc += V[i]*V[i];
    }

    acc = 1.0 / Math.sqrt(acc);
    for (var i = 0; i < N; i++) {
        V[i] *= acc;
    }

    return V;
}

function vec_copy(A)
{
    return A.slice();
}

function vec_magnitude(V)
{
    var N = V.length;

    var acc = 0;
    for (var i = 0; i < N; i++) {
        acc += V[i]*V[i];
    }

    return Math.sqrt(acc);
}

function vec_scale(a, scale)
{
    a = a.slice();
    for (var i = 0; i < a.length; i++)
        a[i] *= scale;
    return a;
}

function vec_dotproduct(a, b)
{
    assert(a.length == b.length);

    var acc = 0;
    for (var i = 0; i < a.length; i++)
        acc += a[i]*b[i];
    return acc;
}

function vec_crossproduct(a, b)
{
    assert(a.length == 3 && b.length==3);

    return [ a[1]*b[2] - a[2]*b[1],
             a[2]*b[0] - a[0]*b[2],
             a[0]*b[1] - a[1]*b[0] ];
}

function vec_equals(a, b, eps)
{
    assert(a.length == b.length);

    for (var i = 0; i < a.length; i++)
        if (Math.abs(a[i] - b[i]) > eps)
            return false;
    return true;
}

function vec_add(a, b)
{
    assert(a.length == b.length);

    a = a.slice();
    for (var i = 0; i < a.length; i++)
        a[i] += b[i];
    return a;
}

function vec_subtract(a, b)
{
    assert(a.length == b.length);

    a = a.slice();
    for (var i = 0; i < a.length; i++)
        a[i] -= b[i];
    return a;
}

function vec_scale(a, scale)
{
    a = a.slice();
    for (var i = 0; i < a.length; i++)
        a[i] *= scale;
    return a;
}

function vec_distance(a, b)
{
    assert(a.length == b.length);

    var acc = 0;

    for (var i = 0; i < a.length; i++)
        acc += (a[i]-b[i])*(a[i]-b[i]);

    return Math.sqrt(acc);
}

function vec_make_perpendicular(a, b)
{
    var bdir = vec_normalize(b);
    var dot = vec_dotproduct(a, bdir);
    return vec_subtract(a, vec_scale(bdir, dot));
}

function quat_multiply(a, b)
{
    assert(a.length == 4);
    assert(b.length == 4);

    return mat_create(4, 1, [ a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                              a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                              a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                              a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0] ]);
}

function quat_rotate(q, v)
{
    assert(q.length == 4);
    assert(v.length == 3);

    var t2 = q[0]*q[1];
    var t3 = q[0]*q[2];
    var t4 = q[0]*q[3];
    var t5 = -q[1]*q[1];
    var t6 = q[1]*q[2];
    var t7 = q[1]*q[3];
    var t8 = -q[2]*q[2];
    var t9 = q[2]*q[3];
    var t10 = -q[3]*q[3];

    return mat_create(3, 1, [ 2*((t8+t10)*v[0] + (t6-t4)*v[1]  + (t3+t7)*v[2]) + v[0],
                              2*((t4+t6)*v[0]  + (t5+t10)*v[1] + (t9-t2)*v[2]) + v[1],
                              2*((t7-t3)*v[0]  + (t2+t9)*v[1]  + (t5+t8)*v[2]) + v[2] ]);

}

function angleaxis_to_quat(theta, axis)
{
    assert(axis.length==3);

    axis = vec_normalize(axis);
    var s = Math.sin(theta / 2);
    return mat_create(4, 1, [ Math.cos(theta / 2),
                              axis[0]*s,
                              axis[1]*s,
                              axis[2]*s ]);
}
