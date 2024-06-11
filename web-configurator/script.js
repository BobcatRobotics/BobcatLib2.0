document.getElementById('configForm').addEventListener('submit', function(event) {
    event.preventDefault();

    // Get form data
    const formData = new FormData(event.target);
    const configData = Object.fromEntries(formData.entries());

    // Save to local storage
    localStorage.setItem('swerveDriveConfig', JSON.stringify(configData));

    // Display the saved configuration
    displayConfig(configData);
});



document.getElementById('downloadJson').addEventListener('click', function() {
    const newConfig = JSON.parse(localStorage.getItem('swerveDriveConfig'));
    const existingConfig = JSON.parse(localStorage.getItem('existingConfig')) || {};

    // Merge newConfig into existingConfig
    const updatedConfig = { ...existingConfig, ...newConfig };

    downloadJSON(updatedConfig, 'swerve_drive_config.json');
});

function displayConfig(config) {
    const output = document.getElementById('output');
    output.innerHTML = '<h2>Saved Configuration</h2>';
    Object.entries(config).forEach(([key, value]) => {
        const p = document.createElement('p');
        p.textContent = `${key}: ${value}`;
        output.appendChild(p);
    });
}

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

// Load saved configuration on page load
window.addEventListener('load', () => {
    const savedConfig = localStorage.getItem('swerveDriveConfig');
    if (savedConfig) {
        const configData = JSON.parse(savedConfig);
        displayConfig(configData);
    }
});
