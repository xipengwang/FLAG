////
//  Progress Bar
////
function ProgressBar(id, height, width) {

    this.id = id;
    this.el = document.getElementById(id);
    this.height = height;
    this.width = width;
    this.init();
}

ProgressBar.prototype.init = function() {

    this.border_div = document.createElement('div');
    this.border_div.id = this.id + '-border';
    this.border_div.style.width = this.width + 'px';
    this.border_div.style.height = this.height + 'px';
    this.border_div.style.border = '1px solid black';

    this.progress_div = document.createElement('div');
    this.progress_div.id = this.id + '-progress';
    this.progress_div.style.height = this.height + 'px';
    this.progress_div.style.width = 0 + 'px';
    this.progress_div.style.backgroundColor = 'black'

    this.border_div.appendChild(this.progress_div);
    this.el.appendChild(this.border_div);
}

ProgressBar.prototype.setPercent = function(p) {

    this.progress_div.style.width = (this.width * (p/100)) + 'px';
}

ProgressBar.prototype.hide = function() {

    this.border_div.style.display = 'none';

}

ProgressBar.prototype.show = function() {

    this.border_div.style.display = 'block';
}
////
//  End Progress Bar
////

function clamp(v, min, max)
{
    if (v < min)
	    return min;
    if (v > max)
	    return max;
    return v;
}


// data: a Uint8Buffer
// XXX implement as a DataView?
function DataInput(data)
{
    this.data = data;
    this.datapos = 0;
    this.dv = new DataView(this.data.buffer, this.data.byteOffset);
}

DataInput.prototype.has_more = function()
{
    return this.datapos < this.data.length;
}

DataInput.prototype.read_u8 = function()
{
    var v = this.data[this.datapos++];
    return v;
}

DataInput.prototype.read_N = function(len)
{
    var res = Uint8Array(this.data, this.datapos, len);
    this.datapos += len;
    return res;
}

DataInput.prototype.read_u16 = function()
{
    var v = 0;
    for (var i = 0; i < 2; i++)
        var v = (v<<8) | this.data[this.datapos++];
    return v;
}

// beware: chrome seems to be aggressive in implementing types as signed int32.
DataInput.prototype.read_u32 = function()
{
    var v = 0;
    for (var i = 0; i < 4; i++)
        var v = (v<<8) | this.data[this.datapos++];
    return v;
}

// WARNING: Javascript implementations usually use double-precision
// numbers with only 53 bits.
DataInput.prototype.read_u64 = function()
{
    var v = 0;
    for (var i = 0; i < 8; i++)
        var v = (v<<8) | this.data[this.datapos++];
    return v;
}

DataInput.prototype.read_f32 = function()
{
    if (1) {
        var f = this.dv.getFloat32(this.datapos, false);
        this.datapos += 4;
        return f;
    }

    var a = new Uint8Array(4);
    for (var i = 0; i < 4; i++)
        a[i] = this.data[this.datapos++];
    var dv = new DataView(a.buffer, 0, 4);
    return dv.getFloat32(0);
}

DataInput.prototype.read_f32_array = function(nelements)
{
    if (1) {
        var buffer = new Float32Array(nelements);
        for (var i = 0; i < nelements; i++)
            buffer[i] = this.read_f32();
        return buffer;
    } else {
        // this.datapos must be multiple of 4?
        var buffer = new Float32Array(this.data.buffer, this.datapos, nelements);
        this.datapos += 4 * nelements;
        return buffer;
    }
}

DataInput.prototype.read_f64 = function()
{
    var a = new Uint8Array(8);
    for (var i = 0; i < 8; i++)
        a[i] = this.data[this.datapos++];
    var dv = new DataView(a.buffer, 0, 8);
    return dv.getFloat64(0);
}

DataInput.prototype.read_id = function()
{
    var v = "_"+this.read_u64();
    return v;
}

// returns a Uint8Array
DataInput.prototype.read_N = function(len)
{
    var a = this.data.subarray(this.datapos, this.datapos + len);
    this.datapos += len;
    return a;
}

DataInput.prototype.read_string_u32 = function()
{
    var len = this.read_u32();

    if (len == 0x7fffffff)
        return null;

    var s = String.fromCharCode.apply(null, this.read_N(len));
    return s;
}

function DataOutput(maxlen)
{
    this.data = new Uint8Array(maxlen);
    this.datapos = 0;
}

DataOutput.prototype.write_array = function(arr)
{
    for (var i = 0; i < arr.length; i++)
        this.data[this.datapos++] = arr[i];
}

DataOutput.prototype.write_u8 = function(v)
{
    this.data[this.datapos++] = v;
}

DataOutput.prototype.write_u32 = function(v)
{
    this.data[this.datapos++] = (v >> 24) & 0xff;
    this.data[this.datapos++] = (v >> 16) & 0xff;
    this.data[this.datapos++] = (v >> 8) & 0xff;
    this.data[this.datapos++] = (v >> 0) & 0xff;
}

DataOutput.prototype.write_u64 = function(v)
{
    this.data[this.datapos++] = (v >> 56) & 0xff;
    this.data[this.datapos++] = (v >> 48) & 0xff;
    this.data[this.datapos++] = (v >> 40) & 0xff;
    this.data[this.datapos++] = (v >> 32) & 0xff;
    this.data[this.datapos++] = (v >> 24) & 0xff;
    this.data[this.datapos++] = (v >> 16) & 0xff;
    this.data[this.datapos++] = (v >> 8)  & 0xff;
    this.data[this.datapos++] = (v >> 0)  & 0xff;
}

DataOutput.prototype.write_string_u32 = function(s)
{
    this.write_u32(s.length);
    for (var i = 0; i < s.length; i++) {
        this.write_u8(s.charCodeAt(i));
    }
}

DataOutput.prototype.write_f32 = function(v)
{
    var dv = new DataView(this.data.buffer, this.datapos, 4);
    this.datapos += 4;
    return dv.setFloat32(0, v);
}

DataOutput.prototype.write_f64 = function(v)
{
    var dv = new DataView(this.data.buffer, this.datapos, 8);
    this.datapos += 8;
    return dv.setFloat64(0, v);
}
