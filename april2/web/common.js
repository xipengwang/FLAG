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

// if user clicks and drags on DOM object 'handle', we will move the
// DOM object 'win'. 'handle' can be a child of 'win', or can be 'win'
// itself.
function make_moveable(handle, win)
{
    var _offx = 0, _offy = 0;
    var _saved_onmousemove = null;
    var _saved_onmouseup = null;

    win.style.position = "absolute";
    handle.style.cursor = "pointer";

    handle.onmousedown = function(ev) {
	    ev.preventDefault();
	    _offx = ev.pageX - parseInt(win.style.left);
	    _offy = ev.pageY - parseInt(win.style.top);
	    _saved_onmousemove = document.onmousemove;
	    _saved_onmouseup = document.onmouseup;

	    document.onmouseup = function() {
	        document.onmousemove = _saved_onmousemove;
	        document.onmouseup = _saved_onmouseup;
	    };

	    document.onmousemove = function(ev) {
	        ev.preventDefault();
	        win.style.left = clamp(ev.pageX - _offx, 0, window.innerWidth - parseInt(win.style.width)) + "px";
            console.log(win.style.left);
	        win.style.top = clamp(ev.pageY - _offy, 0, window.innerHeight - parseInt(win.style.height)) + "px";
	    };
    };
}

// if the user double clicks on DOM object 'handle', the DOM object
// 'content' is toggled between visible and hidden.
function make_collapseable(handle, content)
{
    var hidden = 0;

    handle.onclick = function(ev) {
	    hidden = 1 - hidden;
	    if (hidden) {
	        content.style.display = "none";
	    } else {
	        content.style.display = "block";
	    }
    };
}

function collapse(handle)
{
    handle.onclick();
}

function clamp(v, min, max)
{
    if (v < min)
	    return min;
    if (v > max)
	    return max;
    return v;
}
