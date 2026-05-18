# =============================================================
#  spsa_dseuil.py
#  Optimisation de d_seuil uniquement par SPSA
#  P*, I*, D* fixés depuis spsa_optimizer.py
#  Profil : 14 km/h → freinage → 8 km/h → 14 km/h
# =============================================================

import numpy as np
import matplotlib.pyplot as plt
from motor_model import MotorModel
from spsa_optimizer import PIDController, DT

# =============================================================
#  Constantes du profil
# =============================================================
VIT_LIGNE   = 16.0   # km/h
VIT_VIRAGE  =  8.0   # km/h
DIST_LIGNE  = 20.0   # m
DIST_VIRAGE = 10.0   # m
DIST_SORTIE = 20.0   # m

# P*, I*, D* fixés — issus de spsa_optimizer.py
P_FIXED = 2.976
I_FIXED = 0.049
D_FIXED = 0.029

# Distance de freinage minimale théorique
A_FREIN     = 2.0   # m/s²  — placeholder, remplacer par valeur .ino
D_SEUIL_MIN = ((VIT_LIGNE/3.6)**2 - (VIT_VIRAGE/3.6)**2) \
              / (2 * A_FREIN)   # ≈ 2.55 m
D_SEUIL_MAX = D_SEUIL_MIN * 4   

# =============================================================
#  Consigne basée sur la distance et d_seuil
# =============================================================
def get_consigne(dist, d_seuil):
    dist_debut_freinage = max(0.0, DIST_LIGNE - d_seuil)

    if dist < dist_debut_freinage:
        return None             # plein gaz — pas de PID
    elif dist < DIST_LIGNE + DIST_VIRAGE:
        return VIT_VIRAGE       # PID cible 8 km/h
    else:
        return None             # plein gaz en sortie virage
    

# =============================================================
#  Episode de simulation — d_seuil seul
# =============================================================
def run_episode(d_seuil, motor, dt=DT):
    """
    P, I, D fixes — optimise uniquement d_seuil
    Loss = temps total + pénalité si VIT > VIT_VIRAGE à l'entrée du virage
    """
    motor.reset()
    pid = PIDController(P_FIXED, I_FIXED, D_FIXED)
    pid.reset(duty_init=1.0)

    dist_max = DIST_LIGNE + DIST_VIRAGE + DIST_SORTIE
    t        = 0.0
    penalite = 0.0

    while motor.get_cum_dist() < dist_max:
        dist     = motor.get_cum_dist()
        mesure   = motor.get_vitesse_kmh()
        consigne = get_consigne(dist, d_seuil)

        if consigne is None:
            duty = 1.0          # plein gaz
        else:
            duty = pid.compute(consigne, mesure, dt)

        motor.step(duty, dt)
        
        # ITAE uniquement en zone virage
        if DIST_LIGNE <= dist < DIST_LIGNE + DIST_VIRAGE:
            erreur    = abs(VIT_VIRAGE - mesure)
            penalite += t * erreur * dt

        t += dt
        if t > 60.0:
            penalite += 1000.0
            break

    # temps total + erreur virage + pénalité sécurité
    return t + penalite

# =============================================================
#  SPSA — 1 paramètre : d_seuil
# =============================================================
def spsa_dseuil(motor, n_iterations=200, alpha=0.5, c=0.3):
    """
    Optimise d_seuil uniquement
    P, I, D sont fixes
    """
    theta     = np.array([D_SEUIL_MIN * 1.2])
    theta_min = np.array([D_SEUIL_MIN])
    theta_max = np.array([D_SEUIL_MAX])

    historique_loss   = []
    historique_dseuil = []

    print("=" * 50)
    print(f"  SPSA d_seuil — config:{motor.config}")
    print(f"  P={P_FIXED}  I={I_FIXED}  D={D_FIXED}  (fixes)")
    print(f"  d_seuil_min : {D_SEUIL_MIN:.2f} m")
    print(f"  d_seuil_max : {D_SEUIL_MAX:.2f} m")
    print(f"  iterations  : {n_iterations}")
    print("=" * 50)
    print(f"  {'iter':>5}  {'d_seuil':>10}  {'loss':>10}")
    print("-" * 30)

    for k in range(n_iterations):

        ak = alpha / (k + 1) ** 0.602
        ck = c     / (k + 1) ** 0.101

        delta = np.random.choice([-1.0, 1.0], size=1)

        theta_plus  = np.clip(theta + ck * delta, theta_min, theta_max)
        theta_minus = np.clip(theta - ck * delta, theta_min, theta_max)

        loss_plus  = run_episode(theta_plus[0],  motor)
        loss_minus = run_episode(theta_minus[0], motor)

        gradient = (loss_plus - loss_minus) / (2.0 * ck * delta)
        theta    = np.clip(theta - ak * gradient, theta_min, theta_max)

        loss_courante = run_episode(theta[0], motor)
        historique_loss.append(loss_courante)
        historique_dseuil.append(theta[0])

        if k % 10 == 0:
            print(f"  {k:>5}  {theta[0]:>10.3f}  {loss_courante:>10.4f}")

    d_seuil_star = theta[0]
    print("=" * 50)
    print(f"  d_seuil* = {d_seuil_star:.4f} m")
    print(f"  freinage déclenché à dist = {DIST_LIGNE - d_seuil_star:.2f} m")
    print(f"  loss finale = {historique_loss[-1]:.4f}")
    print("=" * 50)

    return d_seuil_star, historique_loss, historique_dseuil

# =============================================================
#  Visualisation
# =============================================================
def plot_resultats(d_seuil_star, motor, historique_loss, historique_dseuil):

    fig, axes = plt.subplots(2, 3, figsize=(16, 8))

    # --- Graphe 1 : convergence loss ---
    axes[0][0].plot(historique_loss, color='steelblue', linewidth=1.2)
    axes[0][0].set_xlabel("iteration")
    axes[0][0].set_ylabel("loss")
    axes[0][0].set_title("Convergence loss")
    axes[0][0].grid(True, alpha=0.3)

    # --- Graphe 2 : convergence d_seuil ---
    axes[0][1].plot(historique_dseuil, color='purple', linewidth=1.2)
    axes[0][1].axhline(y=D_SEUIL_MIN, color='red',   linestyle='--',
                       linewidth=0.8, label=f'd_min={D_SEUIL_MIN:.2f}m')
    axes[0][1].axhline(y=D_SEUIL_MAX, color='orange', linestyle='--',
                       linewidth=0.8, label=f'd_max={D_SEUIL_MAX:.2f}m')
    axes[0][1].set_xlabel("iteration")
    axes[0][1].set_ylabel("d_seuil (m)")
    axes[0][1].set_title("Convergence d_seuil")
    axes[0][1].legend(fontsize=8)
    axes[0][1].grid(True, alpha=0.3)

    # --- Simulation finale ---
    motor.reset()
    pid = PIDController(P_FIXED, I_FIXED, D_FIXED)
    pid.reset(duty_init=1.0)
    dist_max = DIST_LIGNE + DIST_VIRAGE + DIST_SORTIE

    temps, vitesses, consignes, duties, distances = [], [], [], [], []
    t = 0.0
    while motor.get_cum_dist() < dist_max:
        dist     = motor.get_cum_dist()
        mesure   = motor.get_vitesse_kmh()
        consigne = get_consigne(dist, d_seuil_star)

        if consigne is None:
            duty = 1.0
        else:
            duty = pid.compute(consigne, mesure, DT)

        motor.step(duty, DT)
        temps.append(t)
        vitesses.append(mesure)
        consignes.append(consigne if consigne is not None else mesure)
        duties.append(duty)
        distances.append(dist)
        t += DT
        if t > 60.0:
            break

    # --- Graphe 3 vide (placeholder) ---
    axes[0][2].axis('off')

    # --- Graphe 4 : vitesse vs temps ---
    axes[1][0].plot(temps, vitesses,  color='steelblue', linewidth=1.5,
                    label='vitesse simulee')
    axes[1][0].plot(temps, consignes, color='red',       linewidth=1.0,
                    linestyle='--', label='consigne')
    axes[1][0].set_xlabel("temps (s)")
    axes[1][0].set_ylabel("vitesse (km/h)")
    axes[1][0].set_title("Vitesse vs temps")
    axes[1][0].legend(fontsize=8)
    axes[1][0].grid(True, alpha=0.3)

    # --- Graphe 5 : vitesse vs distance ---
    axes[1][1].plot(distances, vitesses,  color='steelblue', linewidth=1.5,
                    label='vitesse simulee')
    axes[1][1].plot(distances, consignes, color='red',       linewidth=1.0,
                    linestyle='--', label='consigne')
    axes[1][1].axvspan(0,          DIST_LIGNE,              alpha=0.05,
                       color='green',  label='ligne droite')
    axes[1][1].axvspan(DIST_LIGNE, DIST_LIGNE + DIST_VIRAGE, alpha=0.05,
                       color='orange', label='virage')
    axes[1][1].axvspan(DIST_LIGNE + DIST_VIRAGE, dist_max,  alpha=0.05,
                       color='green')
    axes[1][1].axvline(x=DIST_LIGNE - d_seuil_star, color='purple',
                       linestyle='--', linewidth=1.0,
                       label=f'd_seuil*={d_seuil_star:.2f}m')
    axes[1][1].set_xlabel("distance (m)")
    axes[1][1].set_ylabel("vitesse (km/h)")
    axes[1][1].set_title("Vitesse vs distance")
    axes[1][1].legend(fontsize=8)
    axes[1][1].grid(True, alpha=0.3)

    # --- Graphe 6 : PWM ---
    axes[1][2].plot(temps, duties, color='darkorange', linewidth=1.0)
    axes[1][2].axhline(y=0, color='gray', linewidth=0.5)
    axes[1][2].set_xlabel("temps (s)")
    axes[1][2].set_ylabel("duty [0, 1]")
    axes[1][2].set_title("Commande PWM")
    axes[1][2].grid(True, alpha=0.3)

    plt.suptitle(
        f"SPSA d_seuil — config:{motor.config}  "
        f"P={P_FIXED}  I={I_FIXED}  D={D_FIXED}  "
        f"d_seuil*={d_seuil_star:.2f}m",
        fontsize=10)
    plt.tight_layout()
    plt.savefig(f"spsa_dseuil_{motor.config}.png", dpi=150)
    plt.show()

# =============================================================
#  Main
# =============================================================
if __name__ == "__main__":

    config = "banc"   # ← changer ici pour "piste"
    motor  = MotorModel(config=config)

    d_seuil_star, historique_loss, historique_dseuil = spsa_dseuil(
        motor,
        n_iterations = 500,
        alpha        = 0.5,
        c            = 0.3
    )

    plot_resultats(d_seuil_star, motor, historique_loss, historique_dseuil)