"use strict";

NodeList.prototype.indexOf = Array.prototype.indexOf;
let $ = (s, o = document) => o.querySelectorAll(s);

function empty(e) {
    while (e.firstChild) {
        e.removeChild(e.firstChild);
    }
}

let ws;

window.addEventListener("load", function(event) {
    init();
    for (let p of $("#intercom > p")) {
        p.addEventListener("click", function(event) {
            let s = event.target.nextSibling.style;
            if (s.display == "none") s.display = "inherit";
            else s.display = "none";
        });
    }
    document.addEventListener("visibilitychange", function() {
        ws.send("[1,0]");
    });
});

function init() {
    ws = new WebSocket("ws:/" + window.location.hostname + "/ws");
    ws.onmessage = processMessage;
    ws.onclose = reconnect;
    ws.onopen = function() {
        ws.send('[7]');
        ws.send('[8]');
        ws.send('[9]');
        ws.send('[10]');
        setTimeout(link, 2000);
    };
    // ws.timeout = setTimeout(function() {
    //     ws.close(); // triggert onclose
    // }, 5000);
}

function reconnect() {
    console.error("WebSocket getrennt. Erneut verbinden in 2 s...");
    clearTimeout(ws.timeout);
    ws.timeout = setTimeout(init, 2000);
}

function processMessage(event) {
    try {
        let json = JSON.parse(event.data);
        let functions = [
            undefined,
            gotStatus,
            gotLog,
            undefined, // keine Befehle empfangbar
            settingResponse,
            parameterResponse,
            gotPv,
            gotCommandList,
            gotSettingList,
            gotParameterList,
            gotPvList
        ];
        functions[json[0]](json[1]);
    } catch (e) {
        console.error(e);
    }
    // clearTimeout(ws.timeout);
    // ws.timeout = setTimeout(reconnect, 2000);
    displayConnectivity();
}

function displayConnectivity() {
    if (displayConnectivity.locked == "undefined") {
        displayConnectivity.locked = false;
    }
    if (displayConnectivity.locked) return;
    displayConnectivity.locked = true;
    setTimeout(function(){
        displayConnectivity.locked = false;
    }, 100);
    let e = $("#ws")[0];
    let spinner = {
        '-' : '\\',
        '\\' : '|',
        '|' : '/',
        '/' : '-',
    };
    e.innerHTML = spinner[e.innerHTML];
}

function gotStatus(message) {
    ws.send("[1,1]");
}

function gotLog(message) {
    let filter = $("[name=logFilter]")[0].value;
    if (!message.includes(filter)) return;
    let log = $("#log")[0];
    let level = message.charAt(0);
    let line = document.createElement("pre");
    line.innerHTML = message;
    let colors = {
        'E' : "red",
        'W' : "orange",
        'I' : "green",
        'D' : "black",
        'V' : "gray"
    };
    line.style.color = colors[level];
    log.prepend(line);
}

function genericCreateForm(json, form, type, value, label) {
    empty(form);
    for (let node of json) {
        let set = document.createElement("fieldset");
        set.name = node[0];
        set.innerHTML = "<legend>" + node[0] + "</legend>";
        for (let element of node[1]) {
            set.innerHTML += "<input type='" + type + "' name='" + element + "' value='" + (value ? value : element) + "'> ";
            if (label) set.innerHTML += "<label for='" + element + "'>" + element + "</label><br>";
        }
        form.appendChild(set);
    }
}

function gotCommandList(commands) {
    let f = $("#commands")[0];
    genericCreateForm(commands, f, "button", undefined, false);
    f.addEventListener("click", commandClick, true);
}

function commandClick(event) {
    let t = event.target;
    let p = t.parentNode;
    let n = $("fieldset", t.form).indexOf(p);
    let e = $("input", p).indexOf(t);
    ws.send("[3,[" + n + "," + e + "]]");
}

function gotSettingList(settings) {
    let f = $("#settings")[0];
    genericCreateForm(settings, f, "number", "0", true);
    $("input", f).forEach(valueRequest);
    f.addEventListener("blur", valueBlur, true);
}

function settingResponse(setting) {
    let f = $("#settings")[0];
    let n = $("fieldset", f)[setting[0]];
    let e = $("input", n)[setting[1]];
    e.setAttribute("qType", setting[2]);
    e.value = setting[3];
    e.dispatchEvent(new Event("change"));
}

function gotParameterList(parameters) {
    let f = $("#parameters")[0];
    genericCreateForm(parameters, f, "number", "0", true);
    $("input", f).forEach(valueRequest);
    f.addEventListener("blur", valueBlur, true);
}

function parameterResponse(parameter) {
    let f = $("#parameters")[0];
    let n = $("fieldset", f)[parameter[0]];
    let e = $("input", n)[parameter[1]];
    e.setAttribute("qType", parameter[2]);
    e.value = parameter[3];
    e.dispatchEvent(new Event("change"));
}

function valueRequest(i) {
    let p = i.parentNode;
    let n = $("fieldset", i.form).indexOf(p);
    let e = $("input", p).indexOf(i);
    ws.send("[" + (i.form.id == "settings" ? 4 : 5) + ",[" + n + "," + e + "]]");
}

function valueBlur(event) {
    let t = event.target;
    let p = t.parentNode;
    let n = $("fieldset", t.form).indexOf(p);
    let e = $("input", p).indexOf(t);
    let v = t.value;
    if (!t.attributes.qType) return;
    switch (parseInt(t.attributes.qtype.value)) {
        case (1):
            if (v < 0) v = 0;
        case (2):
            v = Math.round(v);
        case (3):
            break;
        default:
            return;
    }
    ws.send("[" + (t.form.id == "settings" ? 4 : 5) + ",[" + n + "," + e + "," + t.value + "]]");
}

function gotPvList(pvs) {
    let f = $("#pvs")[0];
    genericCreateForm(pvs, f, "button", "Registrieren", true);
    for (let e of $("input", f)) {
        e.addEventListener("click", pvRegister);
    }
}

function pvRegister(event) {
    let t = event.target;
    let p = t.parentNode;
    let n = $("fieldset", t.form).indexOf(p);
    let e = $("input", p).indexOf(t);
    t.value = 0;
    t.type = "number";
    //t.disabled = true;
    ws.send("[6,[" + n + "," + e + "]]");
}

function gotPv(pv) {
    let f = $("#pvs")[0];
    let n = $("fieldset", f)[pv[0]];
    let e = $("input", n)[pv[1]];
    if (pv[2] == 0) {
        e.type = "text";
        e.value = (new Date()).toLocaleTimeString();
    } else {
        e.value = pv[3];
    }
    e.dispatchEvent(new Event("change"));
}

function clearLog() {
    empty($("#log")[0]);
}

function link() {
    for (let e of $("input[q-link]")) {
        let s = e.getAttribute("q-link").split("/");
        let i = $("#" + s[0] + "s > fieldset[name=" + s[1] + "] > input[name=" + s[2] + "]")[0];
        if (!i) continue;
        if (e.type == "text") e.type = i.type;
        e.value = i.value;
        switch(s[0]) {
            case ("command"):
                e.onclick = () => {
                    i.dispatchEvent(new Event("click"));
                };
                break;
            case ("setting"):
            case ("parameter"):
                e.onblur = () => {
                    i.value = e.value;
                    i.dispatchEvent(new Event("blur"));
                };
                i.addEventListener("change", () => {
                    e.value = i.value;
                });
                break;
            case ("pv"):
                e.type = "number";
                e.disabled = true;
                if (i.value == "Registrieren") i.dispatchEvent(new Event("click"));
                i.addEventListener("change", () => {
                    e.value = i.value;
                });
                break;
        }
    }
    for (let div of $("div[q-log]")) {
        empty(div);
        let attrib = div.getAttribute("q-log");
        let csv = [];
        let a = document.createElement("a");
        a.download = attrib.replace(/[,\/]/g, "-") + ".csv";
        a.style = "display: none";
        div.appendChild(a);
        let toggle = document.createElement("input");
        toggle.type = "button";
        toggle.value = "Ein";
        div.appendChild(toggle);
        toggle.onclick = () => {
            if (div.hasAttribute("q-enabled")) {
                div.removeAttribute("q-enabled");
                let url = window.URL.createObjectURL(new Blob(csv, {type: "text/csv"}));
                a.href = url;
                a.click();
                window.URL.revokeObjectURL(url);
                toggle.value = "Ein";
                stat.value = 0;
                csv = [csv[0]];
            } else {
                div.setAttribute("q-enabled", "true");
                toggle.value = "Aus / Download";
            }
        }
        let stat = document.createElement("input");
        stat.type = "number";
        stat.value = 0;
        stat.disabled = true;
        div.appendChild(stat);
        csv.push("Time;" + attrib.replace(/,/g, ";") + "\n");
        let values = attrib.split(",");
        for (let [index, value] of values.entries()) {
            value = value.split("/");
            let input = $("#" + value[0] + "s > fieldset[name=" + value[1] + "] > input[name=" + value[2] + "]")[0];
            if (!input) continue;
            input.addEventListener("change", () => {
                if (!div.hasAttribute("q-enabled")) return;
                let row = [];
                row.push(Date.now());
                for (let i = 0; i < index; ++i) {
                    row.push("");
                }
                row.push(input.value);
                csv.push(row.join(";") + "\n");
                stat.value = parseInt(stat.value) + 1;
            });
            if (input.value == "Registrieren") input.dispatchEvent(new Event("click"));
        }
    }
}
