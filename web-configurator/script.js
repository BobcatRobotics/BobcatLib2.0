


// Event listener for the moduleForm to save configuration
document.getElementById('moduleForm').addEventListener('submit', function(event) {
    event.preventDefault();

    // Get form data
    const formData = new FormData(event.target);
    const moduleConfigData = Object.fromEntries(formData.entries());

    // Save module configuration to local storage
    localStorage.setItem('moduleConfig', JSON.stringify(moduleConfigData));

    // Optionally, display the saved module configuration
    displayModuleConfig(moduleConfigData);

    // Merge and save both configurations
    saveMergedConfig();
});

// Event listener for the configForm to save configuration
document.getElementById('configForm').addEventListener('submit', function(event) {
    event.preventDefault();

    // Get form data
    const formData = new FormData(event.target);
    const configData = Object.fromEntries(formData.entries());

    // Save swerve drive configuration to local storage
    localStorage.setItem('swerveDriveConfig', JSON.stringify(configData));

    // Display the saved swerve drive configuration
    displayConfig(configData);

    // Merge and save both configurations
    saveMergedConfig();
});

// Event listener for the downloadForm to trigger download
document.getElementById('downloadForm').addEventListener('submit', function(event) {
    event.preventDefault();

    // Download configuration
    downloadConfig();
});

// Function to save the merged configuration
function saveMergedConfig() {
    const moduleConfigData = JSON.parse(localStorage.getItem('moduleConfig')) || {};
    const swerveDriveConfigData = JSON.parse(localStorage.getItem('swerveDriveConfig')) || {};

    // Merge both configurations
    const mergedConfig = { ...moduleConfigData, ...swerveDriveConfigData };

    // Save merged configuration to local storage
    localStorage.setItem('mergedConfig', JSON.stringify(mergedConfig));
}

// Function to download configuration
function downloadConfig() {
    const mergedConfig = JSON.parse(localStorage.getItem('mergedConfig'));
    if (mergedConfig) {
        // Download the configuration JSON
        downloadJSON(mergedConfig, 'merged_config.json');
    }
}

// Load saved configurations on page load
window.addEventListener('load', () => {
    const savedModuleConfig = localStorage.getItem('moduleConfig');
    const savedSwerveDriveConfig = localStorage.getItem('swerveDriveConfig');

    if (savedModuleConfig && savedSwerveDriveConfig) {
        const moduleConfigData = JSON.parse(savedModuleConfig);
        const swerveDriveConfigData = JSON.parse(savedSwerveDriveConfig);
        displayAllConfigurations(moduleConfigData, swerveDriveConfigData);
    }
});


// Function to download JSON
function downloadJSON(data, filename) {
    const jsonStr = JSON.stringify(data, null, 2); // Pretty print JSON
    const blob = new Blob([jsonStr], { type: 'application/json' });
    const url = URL.createObjectURL(blob);

    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
}

// Function to display module configuration
function displayModuleConfig(config) {
    const output = document.getElementById('output');
    output.innerHTML = '<h2>Module Configuration</h2>';
    Object.entries(config).forEach(([key, value]) => {
        const p = document.createElement('p');
        p.textContent = `${key}: ${value}`;
        output.appendChild(p);
    });
}

// Function to display swerve drive configuration
function displayConfig(config) {
    const output = document.getElementById('output');
    output.innerHTML = '<h2>Swerve Drive Configuration</h2>';
    Object.entries(config).forEach(([key, value]) => {
        const p = document.createElement('p');
        p.textContent = `${key}: ${value}`;
        output.appendChild(p);
    });
}

// Function to display all configurations
function displayAllConfigurations(moduleConfig, swerveDriveConfig) {
    const output = document.getElementById('output');
    output.innerHTML = '<h2>All Configurations</h2>';

    // Display module configuration
    output.innerHTML += '<h3>Module Configurations</h3>';
    Object.entries(moduleConfig).forEach(([key, value]) => {
        const p = document.createElement('p');
        p.textContent = `${key}: ${value}`;
        output.appendChild(p);
    });

    // Display swerve drive configuration
    output.innerHTML += '<h3>Other Configuration</h3>';
    Object.entries(swerveDriveConfig).forEach(([key, value]) => {
        const p = document.createElement('p');
        p.textContent = `${key}: ${value}`;
        output.appendChild(p);
    });
}
