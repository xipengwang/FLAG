"use strict";

// parentel is a DOM element.
//
// Its children will be turned into items in the list. Each of the
// children should have a property 'handle', which is a DOM element
// (usually a child of one the corresponding list item) which is the
// user interface widget that the user can click/drag on.

// selectedclass: a CSS class that will be applied to objects when
// beind dragged.
function DraggableList(parentel, els, notselectedclass, selectedclass)
{
    var self = this;
    var dragging = null;

    this.parentel = parentel;
    this.els = els;
    this.notselectedclass = notselectedclass;
    this.selectedclass = selectedclass;

    while (this.parentel.firstChild)
        this.parentel.removeChild(this.parentel.firstChild);

    for (var elidx = 0; elidx < this.els.length; elidx++) {
        this.els[elidx].classList.add(notselectedclass);
    }

    for (var elidx = 0; elidx < this.els.length; elidx++) {

        this.els[elidx].style.position = "absolute";

        this.parentel.appendChild(this.els[elidx]);

        // XXX This currently conflicts with periodic buffer_redraw messages
        // which blow away the manual reordering on the client side.
/*
        this.els[elidx].handle.onmousedown = function(el) {
            return function(e) {
                self.dragging = el;
                el.classList.add(self.selectedclass);
                el.classList.remove(self.notselectedclass);

                var saved_onmouseup = document.onmouseup;
                var saved_onmousemove = document.onmousemove;
                self.update();

                // where was the mouse click with respect to the
                // clicked-on object.
                var offy = e.offsetY;

//                el.style.left = parseInt(self.parentel.style.left) + 15 + "px";
                el.style.zIndex = "1";

                document.onmousemove = function(e) {

                    var el_box = el.getBoundingClientRect();
                    var parentel_box = self.parentel.getBoundingClientRect();

                    el.style.top = (e.pageY - offy - parentel_box.top) + "px";

                    var midy = e.pageY - offy + el_box.height / 2 ;

                    // newidx will be the new position for the dragged item.
                    var newidx = -1;

                    // where do we want to insert this item?
                    var y = parentel_box.top;

                    for (var i = 0; i < self.els.length; i++) {
                        var el_box_i = self.els[i].getBoundingClientRect();
                        var dy = el_box_i.height;
                        var y1 = y + el_box_i.height;

                        if (midy >= y && midy <= y1)
                            newidx = i;

                        y += dy;
                    }

                    if (newidx < 0) // do nothing
                        return;

                    if (el != self.els[newidx]) {
                        // we're going to move some items around. Make
                        // sure that we don't have any ties in
                        // draggable order.
                        var dv = 1.0/256;
                        var minv = self.els[0].getDraggableOrder() + dv;

                        for (var i = 1; i < self.els.length; i++) {
                            //console.log(minv);
                            if (self.els[i].getDraggableOrder() < minv) {
                                self.els[i].setDraggableOrder(minv);
                            } else {
                                minv = self.els[i].getDraggableOrder();
                            }
                            minv += dv;
                        }

                        var el_order = el.getDraggableOrder();
                        var newidx_order = self.els[newidx].getDraggableOrder();
                        el.setDraggableOrder(newidx_order);
                        self.els[newidx].setDraggableOrder(el_order);

                        self.update();
                    }
                };

                document.onmouseup = function(e) {
                    self.dragging.classList.remove(self.selectedclass);
                    self.dragging.classList.add(self.notselectedclass);
                    document.onmousemove = saved_onmousemove;
                    document.onmouseup = saved_onmouseup;

                    var parentel_box = self.parentel.getBoundingClientRect();
                    self.dragging = null;
                    el.style.zIndex = "0";
                    self.update();
                };
            };
        }(self.els[elidx]);
    */
    }

    this.update();
}

DraggableList.prototype.getHeight = function()
{
    var y = 0;

    for (var elidx = 0; elidx < this.els.length; elidx++) {
        var el = this.els[elidx];
        var el_box = el.getBoundingClientRect();

        y += el_box.height;
    }

    return y;
}

DraggableList.prototype.update = function()
{
    var parentel_box = this.parentel.getBoundingClientRect();

    var x = 0;
    var y = 0; // parentel_box.top;

    stable_sort(this.els, function(a, b) { return a.getDraggableOrder() - b.getDraggableOrder(); });

    for (var elidx = 0; elidx < this.els.length; elidx++) {
        var el = this.els[elidx];
        var el_box = el.getBoundingClientRect();

        if (this.els[elidx] != this.dragging) {
            el.style.top = y+"px";
        }

        y += el_box.height;
    }
}
