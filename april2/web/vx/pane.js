"use strict";

// 'whole' is the region that moves when you move the window (the
// whole thing!)  'handle' is the region that: when grabbed, moves the
// window; when clicked, toggles visibility of 'content'.
//
// if 'whole' is null, no dragging is allowed.
function Pane(whole, handle, content)
{
    this.whole = whole;
    this.handle = handle;
    this.content = content;

    this.offx = 0;
    this.offy = 0;
    this.armed = false;
    this.collapsed = false;

    this.content.style.position = "absolute";
    this.handle.style.cursor = "pointer";

    var self = this;

    handle.onmousedown = function(ev) {
        self.armed = true;

        var wholebox = whole.getBoundingClientRect();

	    ev.preventDefault();
	    self.offx = ev.pageX - wholebox.left;
	    self.offy = ev.pageY - wholebox.top;

	    var _saved_onmousemove = document.onmousemove;
	    var _saved_onmouseup = document.onmouseup;

	    document.onmouseup = function() {
	        document.onmousemove = _saved_onmousemove;
	        document.onmouseup = _saved_onmouseup;
	    };

	    document.onmousemove = function(ev) {
	        ev.preventDefault();

            var wholebox = self.whole.getBoundingClientRect();
	        self.whole.style.left = clamp(ev.pageX - self.offx, 0, window.innerWidth - wholebox.width) + "px";
	        self.whole.style.top = clamp(ev.pageY - self.offy, 0, window.innerHeight - wholebox.height) + "px";
	    };
    };

    handle.onmousemove = function(ev) {
        // do not toggle collapseable on drags.
        self.armed = false;
    };

    handle.onmouseup = function(ev) {
        if (!self.armed)
            return;
        self.armed = false;

        self.setCollapsed(!self.collapsed);
    };
}

Pane.prototype.setCollapsed = function(collapsed)
{
    this.collapsed = collapsed;

	if (this.collapsed) {
	    this.content.style.display = "none";
	} else {
	    this.content.style.display = "block";
	}
};
