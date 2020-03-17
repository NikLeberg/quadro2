"use strict";

let ws;

window.onload = init;

function init() {
    ws = new WebSocket("ws:/" + window.location.hostname + "/ws");
    ws.onmessage = processMessage;
    ws.onclose = reconnect;
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
            undefined,
            undefined,
            undefined,
            gotPv
        ];
        functions[json[0]](json[1]);
    } catch (e) {
        // console.error(e);
    }
    // // Timeout
    // clearTimeout(ws.timeout);
    // ws.timeout = setTimeout(reconnect, 2000);
    // Status anzeigen
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
    let e = document.getElementById("ws");
    let spinner = {
        '----------' : '+---------',
        '+---------' : '#+--------',
        '#+--------' : '+#+-------',
        '+#+-------' : '-+#+------',
        '-+#+------' : '--+#+-----',
        '--+#+-----' : '---+#+----',
        '---+#+----' : '----+#+---',
        '----+#+---' : '-----+#+--',
        '-----+#+--' : '------+#+-',
        '------+#+-' : '-------+#+',
        '-------+#+' : '--------+#',
        '--------+#' : '---------+',
        '---------+' : '----------'
    };
    e.innerHTML = spinner[e.innerHTML];
}

function gotStatus(message) {
    ws.send("[1,1]");
}

function gotLog(message) {
    let filter = document.getElementsByName("logFilter")[0].value;
    if (!message.includes(filter)) return;
    let log = document.getElementById("log");
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

function gotPv(pv) {
    return;
}

function clearLog() {
    let log = document.getElementById("log");
    while (log.firstChild) {
        log.removeChild(log.firstChild);
    }
}
