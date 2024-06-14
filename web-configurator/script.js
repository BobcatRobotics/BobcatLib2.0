document.addEventListener('DOMContentLoaded', () => {
    const form = document.querySelector('#moduleForm');
    const configForm = document.querySelector('#configForm');
    const outputContent = document.querySelector('#outputContent');
    const downloadButton = document.querySelector('#wickedcooldownloadbutton');
    const useFocCheckbox = document.querySelector('#use_foc');
    const voltageFFGains = document.querySelector('#voltage_ff_gains');
    const showJson = document.querySelector('#showJson');

    form.addEventListener('submit', event => {
        event.preventDefault();
        updateConfig();
    });

    configForm.addEventListener('submit', event => {
        event.preventDefault();
        updateConfig();
    });

    document.querySelectorAll('input').forEach(input => {
        input.addEventListener('input', updateConfig);
        input.addEventListener('change', updateConfig);
    });

    downloadButton.addEventListener('click', downloadConfig);

    useFocCheckbox.addEventListener('change', () => {
        updateConfig();
    });
    voltageFFGains.addEventListener('change', () => {
        updateConfig();
    });

    function updateConfig() {
        const moduleFormData = new FormData(form);
        const configFormData = new FormData(configForm);
        const config = {};

        moduleFormData.forEach((value, key) => {
            const inputElement = form.querySelector(`[name="${key}"]`);
            config[key] = parseValue(value, inputElement);
        });
        config.voltage_ff_gains = voltageFFGains.checked;
        configFormData.forEach((value, key) => {
            const inputElement = configForm.querySelector(`[name="${key}"]`);
            config[key] = parseValue(value, inputElement);
        });

        config.use_foc = useFocCheckbox.checked;


        const jsonString = JSON.stringify(config, null, 2);
        // Save to local storage for persistence
        localStorage.setItem('config', jsonString);
        if (showJson.checked) {
            outputContent.textContent = jsonString;
        } else {
            outputContent.textContent = "";
        }

    }

    function parseValue(value, inputElement) {
        if (value === '') {
            if (inputElement && inputElement.type === 'number') {
                return 0; // Handle empty number fields as 0
            }
            return ''; // Handle empty non-number fields as empty strings
        }
        if (value === 'true' || value === "on") return true;
        if (value === 'false') return false;
        if (!isNaN(value) && inputElement && inputElement.type === 'number') return Number(value);
        return value;
    }

    function downloadConfig() {
        const jsonString = localStorage.getItem('config');
        const blob = new Blob([jsonString], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'config.json';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }

    // Load and display the saved configuration on page load
    const savedConfig = localStorage.getItem('config');
    if (savedConfig) {
        outputContent.textContent = savedConfig;
    }

    // Initial display based on the checkbox state
    useFocCheckbox.dispatchEvent(new Event('change'));
    voltageFFGains.dispatchEvent(new Event('change'));
});
