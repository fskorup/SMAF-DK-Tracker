let socket;
let scanTimeout = null;
let scanCountdown = null;
let savedSSID = "";

window.addEventListener('load', () => {
    socket = new WebSocket('ws://' + window.location.hostname + '/ws');

    socket.onopen = () => {
        console.log('WebSocket connected');
        requestConfigFromESP();
        // scanWiFi();
        // onRefreshWiFiNetworkList();
    };

    socket.onmessage = (event) => {
        const data = JSON.parse(event.data);
    
        if (data.action === "wifi_list") {
            updateSSIDList(data.ssids, savedSSID);
            cancelScanState();
        } else if (data.action === "save_ack") {
            console.log("Config saved. Restarting...");
    
            // Show success message, hide form
            document.getElementById("success").style.display = "block";
            document.getElementById("success").style.visibility = "visible";
            document.getElementById("content").style.display = "none";
        } else {
            populateForm(data); // handle get_config response
        }
    };
});

function requestConfigFromESP() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ action: "get_config" }));
    }
}

function scanWiFi() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ action: "scan_wifi" }));
    }
}

function sendConfig(config) {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ action: "save_config", ...config }));
    }

    console.log(config);
}

function populateForm(data) {
    savedSSID = data.ssidName || "";

    const select = document.getElementById("ssidNames");
    select.innerHTML = "";

    if (savedSSID) {
        const savedOption = document.createElement("option");
        savedOption.value = savedSSID;
        savedOption.textContent = savedSSID;
        savedOption.selected = true;
        select.appendChild(savedOption);
    }

    document.getElementById("ssidPassword").value = data.ssidPassword || "";
    document.getElementById("mqttServer").value = data.mqttServer || "";
    document.getElementById("mqttServerPort").value = data.mqttServerPort || 1883;
    document.getElementById("mqttUsername").value = data.mqttUsername || "";
    document.getElementById("mqttPassword").value = data.mqttPassword || "";
    document.getElementById("mqttClientId").value = data.mqttClientId || "";
    document.getElementById("mqttTopic").value = data.mqttTopic || "";
    document.querySelector('input[name="rgb"]').checked = data.rgb || false;
    document.querySelector('input[name="buzzer"]').checked = data.buzzer || false;

    // Now trigger scan (after dropdown has 1 saved entry)
    onRefreshWiFiNetworkList();
}

function updateSSIDList(ssids, selectedSSID) {
    const select = document.getElementById("ssidNames");
    select.innerHTML = "";

    let matchFound = false;

    // Check if saved SSID exists in scan results
    ssids.forEach(ssid => {
        if (ssid === selectedSSID) {
            matchFound = true;
        }
    });

    // If not in scan list, add saved one manually at the top
    if (!matchFound && selectedSSID) {
        const savedOption = document.createElement("option");
        savedOption.value = selectedSSID;
        savedOption.textContent = selectedSSID;
        savedOption.selected = true;
        select.appendChild(savedOption);
    }

    // Add scanned SSIDs
    ssids.forEach(ssid => {
        const option = document.createElement("option");
        option.value = ssid;
        option.textContent = ssid;
        if (ssid === selectedSSID) {
            option.selected = true;
        }
        select.appendChild(option);
    });
}

function generateRandomString(length = 12) {
    const chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789';
    let result = '';
    for (let i = 0; i < length; i++) {
      result += chars.charAt(Math.floor(Math.random() * chars.length));
    }
    return result;
}

function populateRandomString(inputId) {
    const input = document.getElementById(inputId);
    if (input) {
        input.value = generateRandomString();
        input.focus();
    } else {
        console.warn(`Input with ID "${inputId}" not found.`);
    }
}

function onRefreshWiFiNetworkList() {
    const refreshButton = document.getElementById("refreshButton");
    const ssidSelect = document.getElementById("ssidNames");
    const uploadConfigButton = document.getElementById("uploadConfig");
    const reset = document.getElementById("reset");

    let secondsLeft = 10;
    if (refreshButton) {
        refreshButton.disabled = true;
        refreshButton.textContent = `Scanning (${secondsLeft})`;
    }
    
    if (ssidSelect) {
        ssidSelect.disabled = true;
    }

    if (uploadConfigButton) {
        uploadConfigButton.disabled = true;
    }

    if (reset) {
        reset.disabled = true;
    }

    scanWiFi();

    // Update countdown
    scanCountdown = setInterval(() => {
        secondsLeft--;
        if (secondsLeft > 0 && refreshButton) {
            refreshButton.textContent = `Scanning (${secondsLeft})`;
        }
    }, 1000);

    // Set timeout to cancel state
    scanTimeout = setTimeout(() => {
        console.warn("Scan timeout — restoring UI");
        cancelScanState();
    }, 10000);
}

function cancelScanState() {
    const refreshButton = document.getElementById("refreshButton");
    const ssidSelect = document.getElementById("ssidNames");
    const uploadConfigButton = document.getElementById("uploadConfig");
    const reset = document.getElementById("reset");

    if (refreshButton) {
        refreshButton.disabled = false;
        refreshButton.textContent = "Refresh";
    }
    if (ssidSelect) {
        ssidSelect.disabled = false;
    }

    if (uploadConfigButton) {
        uploadConfigButton.disabled = false;
    }

    if (reset) {
        reset.disabled = false;
    }

    if (scanTimeout) {
        clearTimeout(scanTimeout);
        scanTimeout = null;
    }
    if (scanCountdown) {
        clearInterval(scanCountdown);
        scanCountdown = null;
    }
}

// Called from HTML: onclick="onSubmitButtonClicked()"
function onSubmit() {
    if (!validateForm()) {
        console.warn("Validation failed. Please fill in all fields.");
        return;
    }

    const config = {
        ssidName: document.getElementById("ssidNames").value || "",
        ssidPassword: document.getElementById("ssidPassword").value || "",
        mqttServer: document.getElementById("mqttServer").value || "",
        mqttServerPort: parseInt(document.getElementById("mqttServerPort").value) || 1883,
        mqttUsername: document.getElementById("mqttUsername").value || "",
        mqttPassword: document.getElementById("mqttPassword").value || "",
        mqttClientId: document.getElementById("mqttClientId").value || "",
        mqttTopic: document.getElementById("mqttTopic").value || "",
        rgb: document.querySelector('input[name="rgb"]').checked,
        buzzer: document.querySelector('input[name="buzzer"]').checked
    };

    sendConfig(config);
}

function validateForm() {
    const requiredFields = [
        { id: "ssidNames", required: true },
        { id: "ssidPassword", required: false },
        { id: "mqttServer", required: true },
        { id: "mqttServerPort", required: true },
        { id: "mqttUsername", required: false },
        { id: "mqttPassword", required: false },
        { id: "mqttClientId", required: true },
        { id: "mqttTopic", required: true }
    ];

    let valid = true;
    let firstInvalidInput = null;

    requiredFields.forEach(field => {
        const input = document.getElementById(field.id);
        const label = input.closest(".input-frame")?.querySelector("label");
        const mark = input.closest(".input-frame")?.querySelector("mark");

        let hasError = false;
        let errorMessage = "";

        const value = input.value.trim();

        if (field.required && value === "") {
            hasError = true;
            errorMessage = "Can't be empty.";
        }

        if (field.id === "mqttServerPort") {
            if (value === "") {
                hasError = true;
                errorMessage = "Can't be empty.";
            } else if (isNaN(parseInt(value))) {
                hasError = true;
                errorMessage = "Must be a number.";
            }
        }

        if (hasError) {
            if (!firstInvalidInput) firstInvalidInput = input;
            valid = false;
            if (label) label.style.color = "red";
            if (mark) {
                mark.textContent = errorMessage;
                mark.style.display = "inline";
            }
        } else {
            if (label) label.style.color = "";
            if (mark) {
                mark.textContent = "";
                mark.style.display = "none";
            }
        }
    });

    if (!valid && firstInvalidInput) {
        firstInvalidInput.scrollIntoView({ behavior: "auto", block: "center" });
        firstInvalidInput.focus();
    }

    return valid;
}

function resetFormFields() {
    // Clear all text inputs
    document.querySelectorAll("input[type='text']").forEach(el => el.value = "");

    // Uncheck all checkboxes
    document.querySelectorAll("input[type='checkbox']").forEach(el => el.checked = true);

    // Reset selects (SSID list)
    const ssidSelect = document.getElementById("ssidNames");
    if (ssidSelect) {
        ssidSelect.innerHTML = "";
    }

    // Clear validation marks
    document.querySelectorAll("mark").forEach(el => {
        el.textContent = "";
        el.style.display = "none";
    });

    // Optional: reset savedSSID
    savedSSID = "";
}