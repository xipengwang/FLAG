"use strict";

function UC5(inbuf)
{
    this.inbuf = inbuf;
    this.inpos = 4;
    this.inlen = this.inbuf.length;
    this.outlen = (this.inbuf[0]<<24) + (this.inbuf[1]<<16) +
        (this.inbuf[2]<<8) + (this.inbuf[3]);
    this.out = new Uint8Array(this.outlen);
    this.outpos = 0;

    this.bits_buf = 0; // 32 bits of data at a time
    this.bits_left = 0;
}

UC5.prototype.bit = function()
{
    if (this.bits_left == 0) {
        this.bits_buf = 0;
        for (var i = 0; i < 4; i++) {
            this.bits_buf <<= 8;
            this.bits_buf |= this.inbuf[this.inpos++];
        }
        this.bits_left = 32;
    }

    var bit = this.bits_buf & 1;
    this.bits_buf >>= 1;
    this.bits_left--;
    return bit;
}

UC5.prototype.bits = function(nbits)
{
    /*    if (this.bits_left >= nbits) {
          var v = this.bits & (( 1 << nbits) - 1);
          this.bits >>= nbits;
          this.bits_left -= nbits;
          return v;
          } */

    var v = 0;
    for (var i = 0; i < nbits; i++)
        v |= this.bit() << i;
    return v;
}

UC5.prototype.varint = function()
{
    var v = 0;
    var shift = 0;

    while (1) {
        var a = this.inbuf[this.inpos++];
        v = v | ((a & 0x7f) << shift);

        if ((a & 0x80) == 0)
            break;

        shift += 7;
    }

    return v;
}

UC5.prototype.literal = function()
{
    var len = this.bits(2) + 1;
    if (len == 4)
        len = this.varint() + 3;

    //    console.log(this.outpos+" lit  " + len);

    for (var i = 0; i < len; i++)
        this.out[this.outpos++] = this.inbuf[this.inpos++];
}

UC5.prototype.copy = function()
{
    var z = this.inbuf[this.inpos++];
    var len = 0;

    if ((z & 0xf0) == 0xf0)
        len = this.varint() + 15;
    else
        len = (z >> 4) + 1;

    var ago = (this.varint() << 4) + (z & 0x0f);

    var offset = this.outpos - ago;

    for (var i = 0; i < len; i++)
        this.out[this.outpos++] = this.out[offset++];
}

UC5.prototype.uncompress = function()
{
    if (this.out.length == 0)
        return;

    this.out[this.outpos++] = this.inbuf[this.inpos++];

    while (this.inpos < this.inlen) {
        var bit = this.bit();
        if (bit) {
            this.literal();
            this.copy();
        } else {
            this.copy();
        }
    }

    if (this.outpos != this.out.length)
        console.log("uc5 decompression error; wrong length");

    var chk = 0;
    for (var i = 0; i < this.outpos; i++) {
        chk += this.out[i];
    }

    return this.out;
}
