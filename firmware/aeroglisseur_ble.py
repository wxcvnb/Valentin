# aeroglisseur_ble.py
# Contrôle aéroglisseur via BLE — ESP32-C3 MicroPython
#
# Matériel :
#   Pin 4  → signal ESC brushless (PWM 50 Hz)
#   Pin 5  → servo gauche/droite  (PWM 50 Hz)
#   Pin 6  → servo avant/arrière  (PWM 50 Hz)
#   Pin 8  → NeoPixel (témoin d'état)
#
# Procédure de calibration/armement ESC Dualsky XC-12-lite :
#   1. Throttle à 100 % (MAX) AVANT la mise sous tension batterie
#   2. Connecter la batterie → l'ESC détecte PWM_MAX → bips
#   3. Abaisser le throttle à 0 % (MIN)
#   4. L'ESC détecte PWM_MIN → bips de confirmation → ESC calibré et armé
#
# États BLE (caractéristique STATUS, notify) :
#   0 = IDLE              rouge   — en attente de connexion
#   1 = THROTTLE_HIGH     orange  — throttle à 100 %, prêt pour batterie
#   2 = WAITING_HIGH_BEEP bleu    — batterie connectée, attente bips MAX
#   3 = THROTTLE_LOW      violet  — abaisser le throttle à 0 %
#   4 = ARMED             vert    — ESC calibré et armé
#
# Commandes ARM (écriture uint8) :
#   1 → IDLE → THROTTLE_HIGH      (vérifie throttle = 100 %)
#   2 → THROTTLE_HIGH → WAITING_HIGH_BEEP
#   3 → WAITING_HIGH_BEEP → THROTTLE_LOW  (force throttle à 0 %)
#   4 → THROTTLE_LOW → ARMED      (vérifie throttle = 0 %)
#   0 → désarmement depuis n'importe quel état

import bluetooth
import neopixel
import struct
import time
from machine import Pin, PWM

# ── NeoPixel témoin d'état ────────────────────────────────────────────────────
np = neopixel.NeoPixel(Pin(8, Pin.OUT), 1)

def led(r, g, b):
    np[0] = (r, g, b)
    np.write()

# ── Constantes PWM ────────────────────────────────────────────────────────────
# Période 50 Hz = 20 ms  →  duty_u16 = (durée_µs / 20 000) × 65 535
PWM_FREQ = 50

# Plage étendue 0,5 ms – 2,5 ms (meilleure résolution que la plage standard 1–2 ms)
# duty_u16 = (durée_ms / 20) × 65 535
ESC_MIN  = 1638   # 0,5 ms  →   0 % throttle  (position basse)
ESC_MAX  = 8192   # 2,5 ms  → 100 % throttle  (position haute)

SERVO_MIN = 1638  # 0,5 ms  → −100 %  (butée gauche / arrière)
SERVO_MID = 4916  # 1,5 ms  →    0 %  (position centrale)
SERVO_MAX = 8192  # 2,5 ms  → +100 %  (butée droite / avant)

# ── Objets PWM ────────────────────────────────────────────────────────────────
esc    = PWM(Pin(4), freq=PWM_FREQ, duty_u16=ESC_MIN)
srv_lr = PWM(Pin(5), freq=PWM_FREQ, duty_u16=SERVO_MID)
srv_fb = PWM(Pin(6), freq=PWM_FREQ, duty_u16=SERVO_MID)

def set_throttle(pct):
    """pct : entier 0–100 — bloqué si ESC non armé"""
    pct = max(0, min(100, pct))
    duty = int(ESC_MIN + (ESC_MAX - ESC_MIN) * pct / 100)
    esc.duty_u16(duty)

def set_servo(pwm_obj, val):
    """val : entier −100 à +100"""
    val = max(-100, min(100, val))
    duty = int(SERVO_MID + (SERVO_MAX - SERVO_MID) * val / 100)
    pwm_obj.duty_u16(duty)

# ── Machine d'états ESC ───────────────────────────────────────────────────────
ESC_STATE = 0
conn_h    = None
ble_connected = False

# 5 états : 0=IDLE 1=THROTTLE_HIGH 2=WAITING_HIGH_BEEP 3=THROTTLE_LOW 4=ARMED
LED_COLORS = {
    0: (20,  0,  0),   # rouge  — IDLE
    1: (20, 10,  0),   # orange — THROTTLE_HIGH
    2: ( 0,  0, 20),   # bleu   — WAITING_HIGH_BEEP
    3: (15,  0, 20),   # violet — THROTTLE_LOW
    4: ( 0, 20,  0),   # vert   — ARMED
}
LABELS = {0: "IDLE", 1: "THROTTLE_HIGH", 2: "WAITING_HIGH_BEEP",
          3: "THROTTLE_LOW", 4: "ARMED"}

def update_state(new_state):
    global ESC_STATE
    ESC_STATE = new_state
    led(*LED_COLORS.get(ESC_STATE, (0, 0, 0)))
    # gatts_write met à jour la valeur stockée (lue par readValue côté HTML)
    ble.gatts_write(status_handle, bytes([ESC_STATE]))
    if ble_connected and conn_h is not None:
        ble.gatts_notify(conn_h, status_handle, bytes([ESC_STATE]))
    print("ESC →", LABELS.get(ESC_STATE, "?"))

# ── UUIDs BLE ─────────────────────────────────────────────────────────────────
# !! Les élèves modifient ces UUIDs pour différencier leur binôme !!
# (modifier aussi les UUIDs dans le fichier HTML correspondant)

SERVICE_UUID   = bluetooth.UUID("a2b3c4d5-0001-4abc-8def-123456789faa")
THROTTLE_UUID  = bluetooth.UUID("a2b3c4d5-0002-4abc-8def-123456789faa")
ARM_UUID       = bluetooth.UUID("a2b3c4d5-0003-4abc-8def-123456789faa")
STATUS_UUID    = bluetooth.UUID("a2b3c4d5-0004-4abc-8def-123456789faa")
SERVO_LR_UUID  = bluetooth.UUID("a2b3c4d5-0005-4abc-8def-123456789faa")
SERVO_FB_UUID  = bluetooth.UUID("a2b3c4d5-0006-4abc-8def-123456789faa")

# ── Initialisation GATT ───────────────────────────────────────────────────────
ble = bluetooth.BLE()
ble.active(True)
ble.gap_advertise(None)

((throttle_handle,
  arm_handle,
  status_handle,
  servo_lr_handle,
  servo_fb_handle),) = ble.gatts_register_services((
    (SERVICE_UUID, (
        (THROTTLE_UUID, bluetooth.FLAG_WRITE),
        (ARM_UUID,      bluetooth.FLAG_WRITE),
        (STATUS_UUID,   bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY),
        (SERVO_LR_UUID, bluetooth.FLAG_WRITE),
        (SERVO_FB_UUID, bluetooth.FLAG_WRITE),
    )),
))

def start_advertising():
    # !! Remplacer par votre prénom (ex: b"Alice") !!
    name = b"Valentin"
    adv  = b'\x02\x01\x06' + bytes([len(name) + 1, 0x09]) + name
    assert len(adv) <= 31
    ble.gap_advertise(100_000, adv)
    print("Publicité BLE active")

# ── Gestionnaire d'événements BLE ─────────────────────────────────────────────
def ble_irq(event, data):
    global ble_connected, conn_h, ESC_STATE

    if event == 1:   # _IRQ_CENTRAL_CONNECT
        conn_h, _, _ = data
        ble_connected = True
        led(0, 5, 20)
        print("Client connecté")
        ble.gatts_notify(conn_h, status_handle, bytes([ESC_STATE]))

    elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
        # Sécurité absolue : coupure moteur et désarmement
        set_throttle(0)
        set_servo(srv_lr, 0)
        set_servo(srv_fb, 0)
        ESC_STATE = 0
        ble_connected = False
        conn_h = None
        led(20, 0, 0)
        print("Client déconnecté — moteur stoppé")
        start_advertising()

    elif event == 3:  # _IRQ_GATTS_WRITE
        _, value_handle = data

        # --- Commande de gaz ---
        if value_handle == throttle_handle:
            raw = ble.gatts_read(throttle_handle)
            if len(raw) == 1:
                if ESC_STATE == 4:  # seulement si ARMED
                    set_throttle(raw[0])
                    print("Throttle :", raw[0], "%")
                else:
                    print("Throttle ignoré — ESC non armé (état", ESC_STATE, ")")

        # --- Commande d'armement ---
        elif value_handle == arm_handle:
            raw = ble.gatts_read(arm_handle)
            if len(raw) == 1:
                cmd = raw[0]
                if cmd == 0:
                    # Désarmement d'urgence — couper le moteur immédiatement
                    set_throttle(0)
                    update_state(0)
                elif cmd == 1 and ESC_STATE == 0:
                    # Étape 1 : forcer le throttle à 100 % pour calibration MAX
                    set_throttle(100)
                    update_state(1)
                elif cmd == 2 and ESC_STATE == 1:
                    # Étape 2 : batterie connectée, maintenir throttle à 100 %
                    update_state(2)
                elif cmd == 3 and ESC_STATE == 2:
                    # Étape 3 : bips MAX entendus → abaisser throttle à 0 % (calibration MIN)
                    set_throttle(0)
                    update_state(3)
                elif cmd == 4 and ESC_STATE == 3:
                    # Étape 4 : bips MIN entendus → ESC calibré et armé
                    update_state(4)

        # --- Servo gauche / droite ---
        elif value_handle == servo_lr_handle:
            raw = ble.gatts_read(servo_lr_handle)
            if len(raw) == 1:
                val = struct.unpack('b', raw)[0]  # signé −128..+127
                set_servo(srv_lr, val)

        # --- Servo avant / arrière ---
        elif value_handle == servo_fb_handle:
            raw = ble.gatts_read(servo_fb_handle)
            if len(raw) == 1:
                val = struct.unpack('b', raw)[0]
                set_servo(srv_fb, val)

# ── Démarrage ─────────────────────────────────────────────────────────────────
ble.irq(ble_irq)
update_state(0)
start_advertising()
print("Aéroglisseur BLE prêt — en attente de connexion")

while True:
    time.sleep(1)
