document.querySelectorAll('input').forEach(input => {
    // Trigger save on both input and change events
    input.addEventListener('input', updateAndSaveConfig);
    input.addEventListener('change', updateAndSaveConfig);
});

// Event listener for the moduleForm to save configuration
document.getElementById('moduleForm').addEventListener('submit', function(event) {
    event.preventDefault();
    updateAndSaveConfig();
});

// Event listener for the configForm to save configuration
document.getElementById('configForm').addEventListener('submit', function(event) {
    event.preventDefault();
    updateAndSaveConfig();
});

// Event listener for the downloadForm to trigger download
document.getElementById('downloadForm').addEventListener('submit', function(event) {
    event.preventDefault();
    downloadConfig();
});

// Function to update and save configuration for both forms
function updateAndSaveConfig() {
    // Collect data from both forms
    const moduleFormData = new FormData(document.getElementById('moduleForm'));
    const configFormData = new FormData(document.getElementById('configForm'));

    // Convert form data to objects
    const moduleConfigData = Object.fromEntries(moduleFormData.entries());
    const swerveDriveConfigData = Object.fromEntries(configFormData.entries());

    // Process checkbox values explicitly
    //This doesn't work properly
    moduleConfigData.use_foc = document.getElementById('use_foc').checked;

    // Save individual configurations to local storage
    localStorage.setItem('moduleConfig', JSON.stringify(moduleConfigData));
    localStorage.setItem('swerveDriveConfig', JSON.stringify(swerveDriveConfigData));
    
    // Merge and save both configurations
    saveMergedConfig();
}

// Function to save the merged configuration
function saveMergedConfig() {
    const moduleConfigData = JSON.parse(localStorage.getItem('moduleConfig')) || {};
    const swerveDriveConfigData = JSON.parse(localStorage.getItem('swerveDriveConfig')) || {};

    // Merge both configurations
    const mergedConfig = { ...moduleConfigData, ...swerveDriveConfigData };

    // Save merged configuration to local storage
    localStorage.setItem('mergedConfig', JSON.stringify(mergedConfig));

    // Display the merged configuration
    displayConfig(mergedConfig);
}

// Function to display module configuration
function displayModuleConfig(config) {
    const output = document.getElementById('outputContent');
    output.innerHTML = ''; // Clear existing content
    Object.entries(config).forEach(([key, value]) => {
        const p = document.createElement('p');
        p.textContent = `${key}: ${value}`;
        output.appendChild(p);
    });
}

// Function to display swerve drive configuration
function displayConfig(config) {
    const output = document.getElementById('outputContent');
    output.innerHTML = ''; // Clear existing content

    const showJsonProps = document.getElementById('showJson').checked;
    if(showJsonProps){
    Object.entries(config).forEach(([key, value]) => {
        const p = document.createElement('p');
        p.textContent = `${key}: ${value}`;
        output.appendChild(p);
    });
    }
}

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

// Function to download configuration
function downloadConfig() {
    const mergedConfig = JSON.parse(localStorage.getItem('mergedConfig'));
    if (mergedConfig) {
        // Download the configuration JSON
        downloadJSON(mergedConfig, 'SwerveConstants.json');
    }
}

// Load saved configurations on page load
window.addEventListener('load', () => {
    const savedModuleConfig = localStorage.getItem('moduleConfig');
    if (savedModuleConfig) {
        const moduleConfigData = JSON.parse(savedModuleConfig);
        displayModuleConfig(moduleConfigData);
    }

    const savedSwerveDriveConfig = localStorage.getItem('swerveDriveConfig');
    if (savedSwerveDriveConfig) {
        const swerveDriveConfigData = JSON.parse(savedSwerveDriveConfig);
        displayConfig(swerveDriveConfigData);
    }

    const mergedConfig = JSON.parse(localStorage.getItem('mergedConfig'));
    if (mergedConfig) {
        displayConfig(mergedConfig);
    }
});
