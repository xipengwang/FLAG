"use strict";

// el is a table element.
function VxLayerManager(vc, el)
{
    this.vc = vc;
    this.el = el;
    this.updatePending = false;

    el.style.fontFamily = "sans-serif";

    var self = this;
    vc.onBufferLayerDrawOrder.push(function () { self.update(); });
    self.update();
}

// rate limit the update
VxLayerManager.prototype.update = function()
{
    var self = this;
    if (!this.updatePending) {
        this.updatePending = true;
        window.setTimeout(function() {
            self.updatePending = false;
            self.realUpdate() },
                          200);
    }
}

VxLayerManager.prototype.realUpdate = function()
{
    var self = this;

    var layer_els = [ ];

    for (var layeridx = 0; layeridx < this.vc.layers.length; layeridx++) {
        var layer = this.vc.layers[layeridx];

        var layer_el = document.createElement("div");
        layer.layer_el = layer_el;
        layer_el.className = "noselect draggable_layer";
        layer_el.style.width="200px";
        layer_els.push(layer_el);

        var layer_handle_el = document.createElement("span");
        layer_el.appendChild(layer_handle_el);
        layer_el.handle = layer_handle_el;
        layer_handle_el.className = "handle";
        layer_handle_el.innerHTML = "&nbsp;&nbsp;&nbsp;";

        var layer_name_el = document.createElement("span");
        layer_name_el.innerHTML = layer.name;
        layer_el.appendChild(layer_name_el);
        layer_name_el.style.width="120px";
        layer_name_el.style.display="inline-block";

        layer.updateNameElement = function(layer, layer_name_el) {
            return function() {
                layer_name_el.innerHTML = layer.name + " ("+layer.draworder+")";
                layer_name_el.style.color = layer.visible ? "#000" : "#999";
                layer_name_el.style.textDecoration = layer.visible ? "none" : "line-through";
            }
        }(layer, layer_name_el);

        layer.updateNameElement();

        layer_el.getDraggableOrder = function(layer) {
            return function() { return layer.draworder; };
        } (layer);

        layer_el.setDraggableOrder = function(layer) {
            return function(order) { layer.draworder = order;
                                     layer.updateNameElement();
                                     vc.requestRedraw(); };
        } (layer);

        layer_name_el.onclick = function (buffer) {
            return function() {
                layer.visible ^= true;
                layer.vc.requestRedraw();
                self.update();
            }
        }(layer);

        if (true) {
//
//            var layer_menu_button_el = document.createElement("span");
//            layer_menu_button_el.innerHTML = "&gt;";
//            layer_el.appendChild(layer_menu_button_el);
//
//            var layer_menu_el = document.createElement("span");
//            layer_menu_el.style.position= "fixed";
//
//
//            var menu_el = menu_submenu_create(null, ">");
//            menu_el.className = "menu";
//            menu_el.appendChild(menu_item_create(menu_el, "Item 1"));
//            menu_el.appendChild(menu_item_create(menu_el, "Item 2"));
//            menu_el.appendChild(menu_checkbox_item_create(menu_el, "Checkbox 1", true));
//            menu_el.appendChild(menu_checkbox_item_create(menu_el, "Checkbox 2", false));
//            layer_el.appendChild(menu_submenu_create(null, ">", menu_el));
//
//            layer_el.appendChild(layer_menu_el);/
//            //        layer_menu_el.style.position="fixed";
//            layer_menu_el.style.display="inline-block";
//            //        layer_menu_el.style.width="100px";
//
//
//            var menu = new VxMenu(layer_menu_button_el, layer_menu_el);
//            new VxMenuItem(menu, "one");
//            new VxMenuItem(menu, "two");
//            new VxMenuCheckbox(menu, "check", true);
//            new VxMenuCheckbox(menu, "nocheck", false);
//
//            var ifacemenu = new VxMenu(layer_menu_button_el
//
        }


        var layer_content_el = document.createElement("span");
        layer_el.content = layer_content_el;
        layer_el.appendChild(layer_content_el);
        layer_content_el.style.position = "fixed";
        layer_content_el.style.display = "inline-block";
    }

    // create the outer menu (for layers) so that all objects are in
    // the DOM so sizing works properly.
    new DraggableList(this.el, layer_els, "draggable_not_selected", "draggable_selected");

    for (var layeridx = 0; layeridx < this.vc.layers.length; layeridx++) {
        var layer = this.vc.layers[layeridx];

        if (true) {
            var buffer_els = [ ];

            for (var bufferidx = 0; bufferidx < layer.buffers.length; bufferidx++) {
                var buffer = layer.buffers[bufferidx];

                var buffer_el = document.createElement("div");
                buffer_el.className = "noselect draggable_buffer";
                buffer_els.push(buffer_el);

                var buffer_handle_el = document.createElement("span");
//                buffer_handle_el.innerHTML = "DRAG";
                buffer_el.appendChild(buffer_handle_el);
                buffer_el.handle = buffer_handle_el;
                buffer_handle_el.className = "handle";
                buffer_handle_el.innerHTML = "&nbsp;&nbsp;&nbsp;";

                var buffer_name_el = document.createElement("span");
                buffer_name_el.innerHTML = buffer.name + "("+buffer.draworder+")";
                buffer_el.appendChild(buffer_name_el);
                buffer_name_el.style.width="120px";
                buffer_name_el.style.display="inline-block";

                buffer.updateNameElement = function(buffer, buffer_name_el) {
                    return function() {
                        buffer_name_el.innerHTML = buffer.name + " ("+buffer.draworder+")";
                        buffer_name_el.style.color = buffer.visible ? "#000" : "#999";
                        buffer_name_el.style.textDecoration = buffer.visible ? "none" : "line-through";
                    }
                }(buffer, buffer_name_el);

                buffer.updateNameElement();

                buffer_el.getDraggableOrder = function(buffer) {
                    return function() { return buffer.draworder; };
                } (buffer);
                buffer_el.setDraggableOrder = function(buffer) {
                    return function(order) { buffer.draworder = order;
                                             buffer.updateNameElement();
                                             vc.requestRedraw(); };
                } (buffer);

                var buffer_handle_el = document.createElement("span");
                buffer_name_el.onclick = function (buffer) {
                    return function() {
                        buffer.visible ^= true;
                        layer.vc.requestRedraw();
                        self.update();
                    }
                }(buffer);

            }

            var menu = new DraggableList(layer_els[layeridx].content, buffer_els,
                                         "draggable_not_selected", "draggable_selected");

            layer.layer_el.style.height = menu.getHeight()+"px";
        }
    }

/*
    cell1.onclick = function(vl) {
		return function() {
		vl.visible = !vl.visible;
        for (var j = vl.buffers.length - 1; j >= 0; j--) {
			    var vb = vl.buffers[j];
			    vb.visible = !vl.buffers[0].visible;
		    }

		    update(_vl, _vb);
		    vc.setDirty();
		};
	}(vl);

	var cell2 = tr.insertCell(1);
	cell2.innerHTML = "persp";
	cell2.style.fontStyle = "italic";

	cell2.onclick = function(vl, cell2) {
		return function() {
		    var mtime = new Date().getTime();
		    var current_p = vl.projectionComputer.compute(0, 0, mtime).perspectiveness;
		    var next_p = (current_p != 0) ? 0 : 1;
		    if (next_p == 1)
			    cell2.innerHTML = "persp";
		    else
			    cell2.innerHTML = "ortho";

		    vl.projectionComputer = new InterpolatingProjectionComputer(current_p, vl.default_fovy_degrees,
										                                vl.default_zclip_near, vl.default_zclip_far, mtime,
										                                next_p, vl.default_fovy_degrees,
										                                vl.default_zclip_near, vl.default_zclip_far, mtime+500,
										                                vl);

		    vc.setDirty();
		};
	}(vl, cell2);

	var cell3 = tr.insertCell(2);
	cell3.innerHTML = vl.event_handlers[1].interfaceMode; // XXX won't update if modified programatically
	cell3.style.fontStyle = "italic";
	cell3.onclick = function(vl, cell3) {
		return function() {
		    var eh = vl.event_handlers[1];
		    if (eh.interfaceMode == "3D")
			    eh.interfaceMode = "2.5D";
		    else if (eh.interfaceMode == "2.5D")
			    eh.interfaceMode = "2D";
		    else if (eh.interfaceMode == "2D")
			    eh.interfaceMode = "3D";

		    cell3.innerHTML = eh.interfaceMode;
		    vc.onDirty();
		};
	}(vl, cell3);

	if (vl.visible) {
		for (var j = 0; j < vl.buffers.length; j++) {
		    var vb = vl.buffers[j];

		    var tr = tbl.insertRow(-1);
		    var cell1 = tr.insertCell(0);
		    var cell2 = tr.insertCell(1);
		    cell2.innerHTML = vb.name;

		    if (vb.visible)
			    cell2.style.color = "#000000";
		    else
			    cell2.style.color = "#999999";

		    cell2.onclick = function(vb) {
			    return function() {
			        vb.visible = !vb.visible;
			        update(_vl, _vb);
			        vc.onDirty();
			    };
		    }(vb);
		}
	}
*/
};
