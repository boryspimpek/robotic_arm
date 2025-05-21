const track = document.getElementById('scrollTrack');
const buttons = track.querySelectorAll('button');

let gripperState = 'closed';

const postRequest = (url, body = null) => {
    const options = {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
    };
    if (body) options.body = JSON.stringify(body);
    return fetch(url, options);
};

const setStatus = msg => {
    const el = document.getElementById('status');
    if (el) el.innerText = msg;
};

const toggleGripper = () => {
    const action = gripperState === 'closed' ? 'open_gripper' : 'close_gripper';
    const newState = gripperState === 'closed' ? 'opened' : 'closed';

    postRequest(`/${action}`)
        .then(response => response.text())
        .then(msg => {
            alert(msg);
            gripperState = newState;
        })
        .catch(() => alert(`Błąd przy ${action === 'open_gripper' ? 'otwieraniu' : 'zamykaniu'} grippera.`));
};

const moveServo = (id, angle) => {
    document.getElementById(`angle${id}`).innerText = angle;
    postRequest('/move_servo', { id, angle });
};

const goHome = () => {
    setStatus("Przesuwanie do pozycji domowej...");
    postRequest('/home_position')
        .then(r => r.text())
        .then(seconds => {
            const ms = parseFloat(seconds) * 1000;
            setTimeout(() => {
                updateSliders();
                setStatus("Pozycja domowa ustawiona.");
            }, ms + 200);
        });
};

const movePreset = presetName => {
    setStatus(`Przesuwanie do pozycji ${presetName}...`);
    postRequest(`/move_preset/${presetName}`)
        .then(r => r.text())
        .then(seconds => {
            const ms = parseFloat(seconds) * 1000;
            setTimeout(() => {
                updateSliders();
                setStatus(`Pozycja ${presetName} ustawiona.`);
            }, ms + 200); // dodajemy lekki bufor
        });
};

const savePreset = presetName => {
    setStatus(`Zapisuję preset ${presetName}...`);
    postRequest(`/save_preset/${presetName}`)
        .then(r => r.text())
        .then(msg => {
            setStatus(msg);
        });
};

const playSequence = () => {
    setStatus("Odtwarzam zapisane pozycje...");
    postRequest('/play_sequence')
        .then(r => r.text())
        .then(msg => {
            setStatus(msg);
            updateSliders();
        })
        .catch(err => {
            setStatus("Błąd przy odtwarzaniu sekwencji.");
            console.error(err);
        });
};

const updateSliders = () => {
    fetch('/get_angles')
        .then(response => response.json())
        .then(data => {
            Object.entries(data).forEach(([id, angle]) => {
                const slider = document.getElementById(`slider${id}`);
                const angleDisplay = document.getElementById(`angle${id}`);
                if (slider) slider.value = angle;
                if (angleDisplay) angleDisplay.innerText = angle;
            });
        });
};

const startPad = () => postRequest('/start_pad');

const stopPad = () => {
    postRequest('/stop_pad')
        .then(response => {
            if (response.ok) {
                updateSliders();
                alert('Sterowanie padem wyłączone.');
            } else {
                alert('Błąd podczas wyłączania pada.');
            }
        })
        .catch(() => alert('Błąd połączenia z serwerem.'));
};

const savePosition = () => {
    postRequest('/save_position')
        .then(response => response.text())
        .then(alert)
        .catch(() => alert('Błąd przy zapisie.'));
};

const resetPositions = () => {
    postRequest('/reset_positions')
        .then(response => response.text())
        .then(alert)
        .catch(() => alert('Błąd przy usuwaniu.'));
};

const playPositions = () => {
    postRequest('/play_positions')
        .then(response => response.text())
        .then(msg => {
            alert(msg);
            waitForPlayDone();
        })
        .catch(() => alert('Błąd przy odtwarzaniu.'));
};

const waitForPlayDone = () => {
    const interval = setInterval(() => {
        fetch('/play_status')
            .then(res => res.text())
            .then(text => {
                if (text === 'done') {
                    clearInterval(interval);
                    updateSliders();
                    alert('Sekwencja zakończona');
                }
            });
    }, 1000);
};

const stopPositions = () => {
    postRequest('/stop_positions')
        .then(response => response.text())
        .then(msg => {
            alert(msg);
            setTimeout(updateSliders, 2000);
        })
        .catch(() => alert('Błąd przy zatrzymywaniu sekwencji.'));
};

const torqueOn = () => {
    postRequest('/torque_on')
        .then(response => response.text())
        .then(alert)
        .catch(() => alert('Błąd przy włączaniu torque'));
};

const torqueOff = () => {
    postRequest('/torque_off')
        .then(response => response.text())
        .then(alert)
        .catch(() => alert('Błąd przy wyłączaniu torque'));
};

const enableManualControl = () => {
    postRequest('/start_manual')
        .then(r => r.text())
        .then(seconds => {
            const ms = parseFloat(seconds) * 1000;

            [1, 2, 3, 4].forEach(id => {
                const slider = document.getElementById(`slider${id}`);
                if (slider) slider.disabled = false;
            });

            setStatus("Sterowanie ręczne aktywne");

            setTimeout(() => {
                updateSliders();
                setStatus("Pozycja startowa gotowa – steruj ręcznie");
            }, ms + 200);
        })
        .catch(() => alert("Błąd podczas uruchamiania sterowania ręcznego."));
};

const moveToPoint = (costMode = 'standard') => {
    const x = parseFloat(document.getElementById('inputX').value);
    const y = parseFloat(document.getElementById('inputY').value);
    const z = parseFloat(document.getElementById('inputZ').value);

    if (isNaN(x) || isNaN(y) || isNaN(z)) {
        alert("Wprowadź poprawne wartości X, Y, Z");
        return;
    }

    postRequest('/move_to_point', { x, y, z, cost_mode: costMode })
        .then(response => response.text())
        .then(text => {
            alert(text);
            const duration = parseFloat(text);
            if (!isNaN(duration)) {
                setTimeout(updateSliders, duration * 1000);
            }
        })
        .catch(() => alert("Błąd podczas ruchu do punktu."));
};

window.onload = () => {
    fetch('/get_angles')
        .then(response => response.json())
        .then(data => {
            Object.entries(data).forEach(([id, angle]) => {
                const slider = document.getElementById(`slider${id}`);
                const angleDisplay = document.getElementById(`angle${id}`);
                if (slider) slider.value = angle;
                if (angleDisplay) angleDisplay.innerText = angle;

                if (slider) {
                    slider.oninput = () => moveServo(id, slider.value);
                }
            });
        });
};






function updateActiveButton() {
    let centerX = window.innerWidth / 2;

    let closestBtn = null;
    let closestDistance = Infinity;

    buttons.forEach(btn => {
        const rect = btn.getBoundingClientRect();
        const btnCenter = rect.left + rect.width / 2;
        const distance = Math.abs(btnCenter - centerX);

        if (distance < closestDistance) {
            closestDistance = distance;
            closestBtn = btn;
        }
    });

    buttons.forEach(btn => btn.classList.remove('active'));
    if (closestBtn) {
        closestBtn.classList.add('active');
    }
}

document.querySelector('.center-select').addEventListener('scroll', () => {
    requestAnimationFrame(updateActiveButton);
});

window.addEventListener('load', updateActiveButton);