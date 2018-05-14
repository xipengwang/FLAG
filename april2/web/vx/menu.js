"use strict";

function menu_create()
{
}

function menu_item_create(menu, name)
{
    var el = document.createElement("div");
    el.innerHTML = name;

    el.onclick = function() {
        if (menu)
            menu.menuclose();
    };

    return el;
}

function menu_submenu_create(menu, name, menu_el)
{
    var el = document.createElement("div");
    var name_el = document.createElement("span");
    name_el.innerHTML = name;
    el.appendChild(name_el);
    el.appendChild(menu_el);
    menu_el.style.display = "none";

    var visible = false;

    name_el.onclick = function() {
        visible ^= true;
        menu_el.style.display = visible ? "inline-block" : "none";
        menu_el.style.zIndex = 100;
//        handle.style.background = visible ? "#555" : "#ddd";
    }

    return el;
}

function menu_checkbox_item_create(menu, name, state)
{
    var el = document.createElement("div");

    var check_el = document.createElement("span");
    check_el.innerHTML = ""+state;
    el.appendChild(check_el);

    var name_el = document.createElement("span");
    name_el.innerHTML = name;
    el.appendChild(name_el);

    el.onclick = function() {
        state ^= true;
        check_el.innerHTML = ""+state;
        menu_el.style.display = visible ? "inline-block" : "none";
        menu_el.style.zIndex = 100;
    }

    return el;
}

function menu_create(handle)
{
    var el = document.createElement("span");
    var visible = false;

    handle.appendChild(el);

    el.style.display = "none";
    el.style.background = "#555";
    el.style.padding = "2px";

    handle.onclick = function() {
        visible ^= true;
        el.style.display = visible ? "inline-block" : "none";
        el.style.zIndex = 100;
        handle.style.background = visible ? "#555" : "#ddd";
    }

    return el;
}

function VxMenu(handle, el)
{
    this.el = el;
    var visible = false;

    el.style.display = "none";
    el.style.background = "#555";
    el.style.padding = "2px";

    handle.onclick = function() {
        visible ^= true;
        el.style.display = visible ? "inline-block" : "none";
        el.style.zIndex = 100;
        handle.style.background = visible ? "#555" : "#ddd";
    }


}

function VxMenuItem(menu, name)
{
    this.el = document.createElement("div");
    this.el.innerHTML = name;

    menu.el.appendChild(this.el);
}

function VxMenuCheckbox(menu, name, state)
{
    this.el = document.createElement("div");
    this.checkmark_el = document.createElement("span");
    this.checkmark_el.innerHTML = ""+state+"&nbsp;";
    this.el.appendChild(this.checkmark_el);

    this.name_el = document.createElement("span");
    this.name_el.innerHTML = name;
    this.el.appendChild(this.name_el);

    menu.el.appendChild(this.el);

}
