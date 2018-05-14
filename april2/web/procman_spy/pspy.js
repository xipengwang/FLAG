"use strict";

var name_inds = [" ","group","name", "enabled","running","cmdline","host", "pid","restarts","last_exit_code","auto_restart","restart_delay_ms"];

function ProcmanSpy()
{
    //this.draws = new Object();
    var pspy = this;

    var mode = "ws://";
    //if (document.location.href.match('/https/g'))
        //mode = "wss://";

    this.ws = new WebSocket(mode + document.location.host + document.location.pathname + "websocket");
    this.ws.binaryType = "arraybuffer";


    this.ws.onopen = function() {
        console.log("web socket opened.");
    };

    this.ws.onmessage = function(event) {
        pspy_on_message(pspy, event.data);
    }

    this.ws.onclose = function() {
        console.log("Connection lost.");
    }

    document.title = "PROC:" + location.hostname;

    this.table = document.getElementById('table');
    for(let i = 0; i < table.rows[0].cells.length; i++){
        pspy.table.rows[0].cells[i].onmouseup = function(e)
        {
            var table = document.getElementById('table');
            var setClass = "active"
            pspy.sortReverse = false;
            if(this.className == "active")
            {
                setClass = "active-flip";
                pspy.sortReverse = true;
            }

            for(var j = 0; j < table.rows[0].cells.length; j++){
                table.rows[0].cells[j].className = "";
            }
            this.className = setClass;

            if(name_inds[i] == "group" ||
               name_inds[i] == "name" ||
               name_inds[i] == "cmdline" ||
               name_inds[i] == "host")
            {
                pspy.sortFunc = pspy.charSort
            }
            else
            {
                pspy.sortFunc = pspy.numSort
            }
            pspy.sortCol = i;

            pspy.sortFunc();

        };
    }

    this.table.rows[0].cells[name_inds.indexOf('group')].onmouseup(null);
    this.sort = setInterval(pspy.sortFunc, 1000);

    document.getElementById('startBtn').onmouseup = function(e){ pspy.handleCommand(e, true)}
    document.getElementById('stopBtn').onmouseup = function(e){ pspy.handleCommand(e, false)}
    document.getElementById('scrollBtn').onmouseup = function(e){ e.target.classList.toggle("clicked")}
    document.getElementById('stdErr').value = "";
    document.getElementById('stdOut').value = "";


    this.last_message = Date.now();
    this.timeout = setInterval(function() {
        var time_since = Date.now() - this.last_message;
        if ((time_since) > 2000) {
            document.getElementById("timeoutwindow").style.display = "block";
            document.getElementById("timeoutwindowtext").innerText = "Disconnected for " + Math.floor((time_since) / 1000) + "s";
        }
        else {
            document.getElementById("timeoutwindow").style.display = "none";
        }

    }.bind(this), 500);

    document.body.onkeyup = function(e) {
        if(e.keyCode == 27) { //ESC
            for(var k = 1; k < pspy.table.rows.length; k++)
            {
                pspy.table.rows[k].classList.remove('clicked');
            }
            var douts = new DataOutput(128);

            douts.write_u32(0x78563412); // magic
            douts.write_u32(202); // PROC_USUB

            if (pspy.ws.readyState == pspy.ws.OPEN)
                pspy.ws.send(douts.data.subarray(0, douts.datapos));

            pspy.lastProcID = -1;
            document.getElementById('stdOut').value = "";
        }
    }
}

ProcmanSpy.prototype.handleCommand = function(e, start)
{
    for(var k = 1; k < pspy.table.rows.length; k++)
    {
        if(pspy.table.rows[k].classList.contains('clicked'))
        {
            var douts = new DataOutput(128);

            douts.write_u32(0x78563412); // magic
            douts.write_u32(200); // ON_DRAW
            douts.write_u32(pspy.table.rows[k].procid);
            douts.write_u8((start == true ? 1 : 0));

            if (pspy.ws.readyState == pspy.ws.OPEN)
                pspy.ws.send(douts.data.subarray(0, douts.datapos));
        }
    }
}

ProcmanSpy.prototype.charSort = function()
{
    var rowData = this.table.getElementsByTagName('tr');

    for(var i = 1; i < rowData.length - 1; i++) {
        for(var j = 1; j < rowData.length - 1; j++) {
            var a = rowData.item(j).getElementsByTagName('td').item(pspy.sortCol).innerHTML;
            var b = rowData.item(j+1).getElementsByTagName('td').item(pspy.sortCol).innerHTML;

            if((a < b && pspy.sortReverse == true) || (a > b && pspy.sortReverse == false))
            {
                    rowData.item(j).parentNode.insertBefore(rowData.item(j+1),rowData.item(j));
            }
        }
    }

}

ProcmanSpy.prototype.numSort = function()
{
    var rowData = pspy.table.getElementsByTagName('tr');

    for(var i = 1; i < rowData.length - 1; i++) {
        for(var j = 1; j < rowData.length - 1; j++) {
            var a = rowData.item(j).getElementsByTagName('td').item(pspy.sortCol).innerHTML;
            var b = rowData.item(j+1).getElementsByTagName('td').item(pspy.sortCol).innerHTML;

            if((a-b < 0 && pspy.sortReverse == true) || (a-b > 0 && pspy.sortReverse == false))
            {
                    rowData.item(j).parentNode.insertBefore(rowData.item(j+1),rowData.item(j));
            }
        }
    }

}

ProcmanSpy.prototype.render = function()
{
    var light_red = "#FFA0A0";
    var light_gray = "#C0C0C0";
    var orange = "#FFA000";
    for (var i = 1; i < this.table.rows.length; i++) {
        var row = this.table.rows[i];
        row.classList.remove('odd');
        row.classList.add((i % 2 == 0) ? "even" : "odd");

        if(row.restarts.textContent != 0)
            row.restarts.style.backgroundColor = light_red;

        var en = row.enabled.textContent;
        var running = row.running.textContent;

        if(running == 1) {
            if(en == 1) {
                row.running.style.backgroundColor = "inherit";
                row.enabled.style.backgroundColor = "inherit";
            } else {
                row.running.style.backgroundColor = orange;
                row.enabled.style.backgroundColor = orange;
            }
        } else {
            if(en == 1) {
                row.running.style.backgroundColor = light_red;
                row.enabled.style.backgroundColor = light_red;
            } else {
                row.running.style.backgroundColor = light_gray;
                row.enabled.style.backgroundColor = light_gray;
            }
        }

    }
}

ProcmanSpy.prototype.getProcID = function(procid)
{
    var proc = undefined;
    for(var j = 1; j < this.table.rows.length; j++) {
        var element = this.table.rows[j];

        if(element.procid == procid) {
            proc = element;
            break;
        }
    }
    return proc;
}

function pspy_on_message(pspy, arraybuffer)
{
    var din = new DataInput(new Uint8Array(arraybuffer));

    var magic = din.read_u32();
    if (magic != 0x21436587)
        return;

    pspy.last_message = Date.now();

    while (din.has_more()) {

        var opcode = din.read_u8();

        if (opcode == 0)
            break;

        switch (opcode) {

            // Process list
            case 100: {
                pspy.procs = [];
                din.read_u64(); //msg length
                din.read_u64(); //utime
                din.read_u64(); //init_utime
                din.read_8(); //exit
                var nproc = din.read_u32();
                for(var i = 0; i < nproc; i++) {
                    //din.read_u64(); //msg length
                    var procid = din.read_u32();
                    var proc = pspy.getProcID(procid);

                    if(proc === undefined) {
                        proc = pspy.table.insertRow();
                        for (var j = 0; j < name_inds.length; j++) {
                            var cell = proc.insertCell();
                            proc[name_inds[j]] = cell;
                            cell.textContent = "";
                        }
                        proc.cmdline.classList.add("wrappable");
                        proc.onmouseup = function(e){
                            var row = e.target.parentNode;
                            var clicked = row.classList.contains('clicked');
                            if(e.ctrlKey == false) {
                                for(var k = 1; k < pspy.table.rows.length; k++)
                                {
                                    pspy.table.rows[k].classList.remove('clicked');
                                }
                            }
                            var pos = row.rowIndex;
                            if(e.shiftKey == true && pspy.lastClick != undefined) {
                                if(pos > pspy.lastClick)
                                    for(var l = pspy.lastClick; l < pos; l++)
                                        pspy.table.rows[l].classList.add('clicked');
                                else
                                    for(var l = pspy.lastClick; l > pos; l--)
                                        pspy.table.rows[l].classList.add('clicked');

                            }
                            pspy.lastClick = pos;

                            if(pspy.lastProcID != row.procid)
                                document.getElementById('stdOut').value = "";

                            var douts = new DataOutput(128);

                            douts.write_u32(0x78563412); // magic
                            if(clicked) {
                                douts.write_u32(202); // PROC_USUB
                                row.classList.remove('clicked');
                            } else {
                                row.classList.add('clicked');
                                douts.write_u32(201); // PROC_SUB
                                douts.write_u32(row.procid);
                            }

                            if (pspy.ws.readyState == pspy.ws.OPEN)
                                pspy.ws.send(douts.data.subarray(0, douts.datapos));

                            pspy.lastProcID = row.procid;
                        }
                    }

                    proc.procid = procid;
                    proc.name.textContent = din.read_string_u32();
                    proc.cmdline.textContent = din.read_string_u32();
                    proc.host.textContent = din.read_string_u32();
                    proc.group.textContent = din.read_string_u32();
                    proc.auto_restart.textContent = din.read_u8();
                    proc.restart_delay_ms.textContent = din.read_u32();
                    proc.enabled.textContent = din.read_u8();
                }
                pspy.render();
                break;
            }
            // Process Status
            case 101: {
                din.read_u64(); //msg length
                din.read_u64(); //utime
                din.read_u64(); //utime
                din.read_u64(); //init_utime
                din.read_string_u32(); //hostname
                var nproc = din.read_u32();
                for(var i = 0; i < nproc; i++) {
                    var procid = din.read_u32();
                    var proc = pspy.getProcID(procid);

                    if(proc === undefined) {
                        if(pspy.table.rows.length > 1)
                            console.log("can't find procid " + procid);
                        din.read_N(13);
                        continue;
                    }

                    proc.running.textContent = din.read_u8();
                    proc.pid.textContent = din.read_u32();
                    proc.restarts.textContent = din.read_u32();
                    proc.last_exit_code.textContent = din.read_32();
                }
                pspy.render();
                break;
            }
            case 102: {
                var procid = din.read_u32();
                var err_stream = din.read_u32();
                var str = din.read_string_u32();
                if(err_stream || procid == pspy.lastProcID) {
                    var autoscroll = document.getElementById('scrollBtn').classList.contains("clicked");
                    if(err_stream) {
                        var proc = pspy.getProcID(procid);
                        var procname = "????????";
                        if(proc != undefined) {
                            procname = proc.name.innerText;
                        }

                        var ta = document.getElementById('stdErr');

                        ta.value += procname + ": " + str;
                        if(ta.value.length > 5000)
                            ta.value = ta.value.slice(-4000);

                        if(autoscroll)
                            ta.scrollTop = ta.scrollHeight;

                    }
                    if(procid == pspy.lastProcID) {
                        var ta = document.getElementById('stdOut');

                        ta.value += str;
                        if(ta.value.length > 5000)
                            ta.value = ta.value.slice(-4000);

                        if(autoscroll)
                            ta.scrollTop = ta.scrollHeight;

                    }

                }

                break;
            }
        }
    }
}
