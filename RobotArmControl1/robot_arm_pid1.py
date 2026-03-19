"""Controladores PID reutilizables para las juntas del brazo robótico."""


class PIDController:
    def __init__(self, gananciaProporcional, gananciaIntegral, gananciaDerivativa, limiteIntegral=200.0, limiteSalida=25.0):
        self.gananciaProporcional = gananciaProporcional
        self.gananciaIntegral = gananciaIntegral
        self.gananciaDerivativa = gananciaDerivativa
        self.limiteIntegral = abs(float(limiteIntegral))
        self.limiteSalida = abs(float(limiteSalida))
        self.integral = 0.0
        self.errorPrevio = 0.0

    def establecerGanancias(self, gananciaProporcional, gananciaIntegral, gananciaDerivativa):
        self.gananciaProporcional = float(gananciaProporcional)
        self.gananciaIntegral = float(gananciaIntegral)
        self.gananciaDerivativa = float(gananciaDerivativa)

    def reiniciar(self):
        self.integral = 0.0
        self.errorPrevio = 0.0

    def actualizar(self, error, intervaloTiempo):
        if intervaloTiempo <= 0:
            return 0.0

        self.integral += error * intervaloTiempo
        self.integral = max(-self.limiteIntegral, min(self.limiteIntegral, self.integral))

        derivada = (error - self.errorPrevio) / intervaloTiempo
        self.errorPrevio = error

        salida = (self.gananciaProporcional * error) + (self.gananciaIntegral * self.integral) + (self.gananciaDerivativa * derivada)
        return max(-self.limiteSalida, min(self.limiteSalida, salida))


class JointPIDManager:
    def __init__(self, configuracionJuntas, pidPorDefecto):
        self.configuracionJuntas = configuracionJuntas
        self.pidPorDefecto = pidPorDefecto
        self.controllers = {}
        self.targets_ref = {}
        self.settled_counts = {}
        self.enabled = False
        self.error_tolerance_deg = 0.6
        self.settle_cycles_required = 3

        for joint_name in self.joints_config:
            defaults = self.pid_defaults[joint_name]
            self.controllers[joint_name] = PIDController(
                kp=defaults["kp"],
                ki=defaults["ki"],
                kd=defaults["kd"],
            )
            self.targets_ref[joint_name] = None
            self.settled_counts[joint_name] = 0

    def set_settling(self, error_tolerance_deg=None, settle_cycles_required=None):
        if error_tolerance_deg is not None:
            self.error_tolerance_deg = max(0.01, float(error_tolerance_deg))
        if settle_cycles_required is not None:
            self.settle_cycles_required = max(1, int(settle_cycles_required))

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        if not self.enabled:
            self.reset_all()

    def reset_all(self):
        for controller in self.controllers.values():
            controller.reset()
        for joint_name in self.settled_counts:
            self.settled_counts[joint_name] = 0

    def set_gains(self, joint_name, kp, ki, kd):
        self.controllers[joint_name].set_gains(kp, ki, kd)

    def set_target(self, joint_name, target_ref_deg, reset_controller=True):
        self.targets_ref[joint_name] = float(target_ref_deg)
        if reset_controller:
            self.settled_counts[joint_name] = 0
            self.controllers[joint_name].reset()

    def clear_target(self, joint_name):
        self.targets_ref[joint_name] = None
        self.settled_counts[joint_name] = 0
        self.controllers[joint_name].reset()

    def get_gains(self, joint_name):
        c = self.controllers[joint_name]
        return c.kp, c.ki, c.kd

    def step(self, current_ref_values, dt):
        if not self.enabled:
            return {}

        outputs = {}
        for joint_name, current_ref in current_ref_values.items():
            target_ref = self.targets_ref.get(joint_name)
            if target_ref is None:
                continue

            error = target_ref - current_ref
            if abs(error) <= self.error_tolerance_deg:
                self.settled_counts[joint_name] += 1
                outputs[joint_name] = target_ref

                # Al llegar estable al setpoint, reseteamos el acumulador para evitar caza.
                if self.settled_counts[joint_name] >= self.settle_cycles_required:
                    self.controllers[joint_name].reset()
                    # Finaliza el control de esta junta para no seguir iterando en bucle.
                    self.targets_ref[joint_name] = None
                    self.settled_counts[joint_name] = 0
                continue

            self.settled_counts[joint_name] = 0
            correction = self.controllers[joint_name].update(error, dt)
            outputs[joint_name] = current_ref + correction

        return outputs
