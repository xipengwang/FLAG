"use strict";

function LCMSpy()
{
	this.labels = ["Channel", "Type", "Received", "Error", "Hz", "Jitter", "Bandwidth"];
	this.properties = ["name", "type", "num", "err", "hz", "jit", "bw"];

    this.ws = new WebSocket("ws://" + document.location.host + document.location.pathname + "websocket");
    this.ws.binaryType = "arraybuffer";
    this.ws.onopen = function() {
        console.log("Web socket opened");
    };
    var spy = this;
    this.ws.onmessage = function(event) {
        spy.onMessage(event.data);
    }
    this.ws.onclose = function() {
        console.log("Connection lost");
        document.getElementById("error-container").style.display = "block";
        if (spy.structViewer.popup && !spy.structViewer.popup.closed)
            spy.structViewer.popup.document.getElementById("error-container").style.display = "block";
    }

    this.channels = null;
    this.activeChannel = null;
    this.sortField = "name";
    this.sortDir = 1;

    this.structViewer = new LCMStructViewer(this);
}

LCMSpy.prototype.onMessage = function(arraybuffer) {
    var din = new DataInput(new Uint8Array(arraybuffer));
    var magic = din.read_u32();
    if (magic != 0x7fc4a1a3)
        return;

    var opcode = din.read_u8();

    if (opcode == 1) {    // Channel list
        var channels = [];
        while (din.has_more()) {
            var row = {};
            row["name"] = din.read_string_u32();
            row["type"] = din.read_string_u32();
            row["hash"] = din.read_u64();
            row["num"] = din.read_u64();
            row["err"] = din.read_u64();
            row["hz"] = din.read_f32().toFixed(2);
            var jit = din.read_f32();
            row["jit"] = (jit < 0) ? "-" : jit.toFixed(2) + " ms";
            row["bw"] = din.read_f32().toFixed(2) + " KiB";
            channels.push(row);
        }
        this.channels = channels;
        this.sort(this.sortField, this.sortDir);
        this.render();
    } else if (opcode == 2) {  // Structure detail
        var msg = JSON.parse(din.read_string_u32());
        this.structViewer.render(msg);
   }
}

LCMSpy.prototype.sort = function(prop, sign) {
    if (this.channels === null)
        return;

    if (!sign)
        sign = 1;
	this.channels.sort(function(a, b) {
		if (a[prop] < b[prop])
			return -sign;
		else if (a[prop] === b[prop])
			return 0;
		else return sign;
	});
}

// Client commands
LCMSpy.prototype.setSubscription = function(channel, bool) {
    var dout = new DataOutput(1024);
    dout.write_u8(bool);
    dout.write_string_u32(channel);
    this.ws.send(dout.data);
}
LCMSpy.prototype.clear = function() {
    var dout = new DataOutput(1);
    dout.write_u8(2);
    this.ws.send(dout.data);
}

LCMSpy.prototype.setActiveChannel = function(channel) {
    // Cancel existing subscription
    if (this.activeChannel) {
        this.setSubscription(this.activeChannel, 0);
        document.getElementById("row:"+this.activeChannel).classList.remove("highlight");
    }

    this.activeChannel = channel;
    if (channel == null) {
        this.structViewer.hide();
    } else {
        this.setSubscription(channel, 1);
        document.getElementById("row:"+channel).classList.add("highlight");

        this.structViewer.show();
        var root = this.structViewer.getRoot();
        if (root)
            root.textContent = "Waiting for first message...";
    }
}

LCMSpy.prototype.makeToggleFn = function(channel) {
    var spy = this;
    return function(e) {
        if (spy.activeChannel === channel) {
            spy.setActiveChannel(null);
        } else {
            if (e.shiftKey)
                spy.structViewer.openWindow();
            spy.setActiveChannel(channel);
        }
    }
}

LCMSpy.prototype.makeSortFn = function(prop) {
    var spy = this;
    return function(e) {
        if (spy.sortField === prop)
            spy.sortDir *= -1;
        else
            spy.sortField = prop;

        spy.sort(spy.sortField, spy.sortDir);
        spy.render();
    }
}

LCMSpy.prototype.render = function() {
	var table = document.createElement("table");
    var header = table.createTHead();
	var row = header.insertRow(0);
	for (var j = 0; j < this.labels.length; j += 1) {
		var col = document.createElement("th");
        col.className = "list-cell";
		col.textContent = this.labels[j];
		row.appendChild(col);
	}

	for (var i = 0; i < this.channels.length; i += 1) {
		var e = this.channels[i];
		var row = table.insertRow(-1);
        row.id = "row:" + e.name;
		row.className = "list-row " + ((i % 2 == 0) ? "even" : "odd");
        if (e.name === this.activeChannel)
            row.className += " highlight";
        row.onclick = this.makeToggleFn(e.name);

		for (var j = 0; j < this.labels.length; j += 1) {
			var col = row.insertCell(j);
            col.id = this.properties[j] + ":" + e.name;
			col.className = "list-cell";
            col.textContent = e[this.properties[j]];
		}
	}

	var root = document.getElementById("list");
    root.replaceChild(table, root.firstChild);
}


function LCMStructViewer(spy) {
    this.spy = spy;
    this.popup = null;
}

LCMStructViewer.prototype.hide = function() {
    if (this.popup !== null) {
        this.popup.close();
        this.popup = null;
    }
    document.getElementById("msg-container").style.display = "none";
}

LCMStructViewer.prototype.openWindow = function() {
    if (this.popup === null || this.popup.closed) {
        this.popup = window.open("struct_viewer.html", "structviewer",
                "location=no,toolbar=no,menubar=no");

        var viewer = this;
        this.popup.onbeforeunload = function() {
            viewer.spy.setActiveChannel(null);
            viewer.popup = null;
        }
    }
}

LCMStructViewer.prototype.show = function() {
    if (this.popup === null || this.popup.closed) {
        document.getElementById("msg-container").style.display = "block";
    } else {
        this.popup.focus();
    }
}

LCMStructViewer.prototype.getRoot = function() {
    if (this.popup === null || this.popup.closed) {
        return document.getElementById("msg");
    }
    return this.popup.document.getElementById("msg");
}

LCMStructViewer.prototype.render = function(msg) {
	var table = document.createElement("table");
	table.className = "struct";
    var row = table.insertRow(-1);
	this.renderRecursive(row, msg);

    var root = this.getRoot();
    root.replaceChild(table, root.firstChild);
}

LCMStructViewer.prototype.renderRecursive = function(row, msg) {
    var type = msg.t;
    if ("d" in msg)
        for (var i in msg.d)
            type += '[' + msg.d[i] + ']';
    var col = row.insertCell(-1);
    col.className = "struct-cell struct-type";
    col.textContent = type;

    col = row.insertCell(-1);
    col.className = "struct-cell struct-value";
    this.renderValue(col, msg.v, msg.o, msg.d);
}

LCMStructViewer.prototype.renderValue = function(root, value, order, dim) {
    if (value === undefined) {
        root.textContent = "...";
        return;
    }else if (value === null) {
        root.textContent = "NaN";
        return;
    }

	var type = Array.isArray(value) ? "array" : typeof value;

	switch (type) {
	case "array":
		var table = document.createElement('table');
		table.className = "struct";
		root.appendChild(table);
		for (var i in value) {
            var row = table.insertRow(-1);
            var col = row.insertCell(0);
            col.className = "struct-index";
			col.textContent = '['+i+']';

            col = row.insertCell(1);
            col.className = "struct-cell struct-value";
			this.renderValue(col, value[i], order, dim.slice(1));
		}
        if (dim && dim[0] > value.length) {
            var row = table.insertRow(-1);
            var col = row.insertCell(0);
            col.className = "struct-index";
			col.textContent = "...";
        }
		break;

	case "object":
		var table = document.createElement('table');
		table.className = "struct";
		root.appendChild(table);
		if (order === undefined)
			order = Object.keys(value);
		for (var i in order) {
			name = order[i];
            var row = table.insertRow(-1);
            var col = row.insertCell(-1);
            col.className = "struct-cell struct-name";
            col.textContent = name;
			this.renderRecursive(row, value[name]);
		}
		break;

	default:
	case "number":
	case "string":
        root.textContent = value;
		break;
	}
}

function HorizontalResize(div) {
    var pane = this;

    this.dragHandler = function(e) {
        div.parentElement.style.width = (pane.startWidth - e.clientX + pane.startX) + "px";
    }
    this.dragEnd = function(e) {
        document.removeEventListener("mousemove", pane.dragHandler, false);
        document.removeEventListener("mouseup", pane.dragEnd, false);
    }
    div.onmousedown = function(e) {
        if (e.button != 0)
            return;
        e.preventDefault();
        pane.startX = e.clientX;
        pane.startWidth = parseInt(window.getComputedStyle(div.parentElement,null).width, 10);

        console.log(pane.startWidth);
        document.addEventListener("mousemove", pane.dragHandler, false);
        document.addEventListener("mouseup", pane.dragEnd, false);
    }
}
