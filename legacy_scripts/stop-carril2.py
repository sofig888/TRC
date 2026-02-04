oxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                if len(approx) == 8 and cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    if 0.7 <= float(w) / h <= 1.3:
                        dist = obtener_distancia_robusta(depth_f, x + w // 2, y + h // 2)
                        memoria_stop.update({"contador": 0, "distancia": dist, "box": (x, y, w, h), "puntos": approx})
                        hallado_stop = True
                        break

            if not hallado_stop:
                memoria_stop["contador"] += 1

            stop_visible = memoria_stop["contador"] < PERSISTENCIA_LIMITE
            d_stop = memoria_stop["distancia"] if stop_visible else 0.0

            # Rearmar STOP cuando se pierde la señal
            if not stop_visible:
                stop_armed = True

            # Intermitentes por cercanía
            if stop_visible and (0 < d_stop < DISTANCIA_INTERMITENTES):
                intermitentes_emergencia = True
            else:
                intermitentes_emergencia = False

            # =====================================================
            # 2) CARRIL AZUL (ROI mitad inferior) - 2 líneas
            # =====================================================
            roi_hsv = hsv[mitad:, :]
            mask_azul = cv2.inRange(roi_hsv, AZUL_LO, AZUL_HI)
            mask_azul = cv2.medianBlur(mask_azul, 5)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, KERNEL, iterations=1)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, KERNEL, iterations=2)

            # separar izquierda/derecha
            halfW = W // 2
            mask_left = mask_azul[:, :halfW]
            mask_right = mask_azul[:, halfW:]

            area_left = int(cv2.countNonZero(mask_left))
            area_right = int(cv2.countNonZero(mask_right))

            ve_2_lineas = (area_left > AREA_MIN_SIDE) and (area_right > AREA_MIN_SIDE)

            lane_center = None
            cx_left = None
            cx_right = None

            if ve_2_lineas:
                M1 = cv2.moments(mask_left)
                M2 = cv2.moments(mask_right)

                if M1["m00"] > 0 and M2["m00"] > 0:
                    cx_left = int(M1["m10"] / M1["m00"])
                    cx_right = int(M2["m10"] / M2["m00"]) + halfW
                    lane_center = int((cx_left + cx_right) / 2)
                else:
                    ve_2_lineas = False

            # Dirección objetivo (mantener centrado)
            direccion_obj = NEUTRO
            if ve_2_lineas and lane_center is not None:
                error = lane_center - (W // 2)

                # OJO: CORREGIDO (antes te giraba al revés)
                direccion_obj = int(NEUTRO - KP_STEER * error)
                direccion_obj = clamp(direccion_obj, IZQ, DER)

            # =====================================================
            # 3) VELOCIDAD objetivo:
            #    - Solo avanza si ve 2 líneas
            #    - Respeta tu bajada progresiva por STOP
            #    - Timer 2s cuando ya se detuvo por completo
            # =====================================================
            # Base por carril
            velocidad_obj = VEL_CRUCERO if ve_2_lineas else NEUTRO

            # --- TU LÓGICA DE BAJAR VELOCIDAD POR STOP (sin cambiar) ---
            if stop_visible:
                d = d_stop
                if 0 < d <= DISTANCIA_PARADA:
                    velocidad_obj = NEUTRO
                elif DISTANCIA_PARADA < d < DISTANCIA_SEGURA:
                    max_permitida = 1500 + (d - DISTANCIA_PARADA) * ((2000 - 1500) / (DISTANCIA_SEGURA - DISTANCIA_PARADA))
                    if velocidad_obj > max_permitida:
                        velocidad_obj = int(max_permitida)

            # --- TIMER: cuando YA está en paro total por STOP, sostener 2s ---
            if stop_armed and stop_visible and (0 < d_stop <= DISTANCIA_PARADA):
                # disparar una sola vez hasta que se pierda el STOP
                stop_hold_until = now + STOP_HOLD_SECONDS
                stop_armed = False

            # si está en hold, forzar neutro sí o sí
            if now < stop_hold_until:
                velocidad_obj = NEUTRO
                pi.write(LED_ROJO, 1)
            else:
                pi.write(LED_ROJO, 0)

            # Aplicar PWM
            pi.set_servo_pulsewidth(ESC_PIN, int(velocidad_obj))
            pi.set_servo_pulsewidth(SERVO_PIN, int(direccion_obj))

            # =====================================================
            # 4) VISUAL (timer visible)
            # =====================================================
            img_draw = img.copy()

            # Dibujo STOP
            if stop_visible and memoria_stop["box"] is not None:
                x, y, w, h = memoria_stop["box"]
                c = (0, 255, 0) if hallado_stop else (0, 255, 255)
                cv2.drawContours(img_draw, [memoria_stop["puntos"]], 0, c, 3)
                cv2.putText(img_draw, f"STOP: {d_stop:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

            # Centro imagen
            cv2.line(img_draw, (W // 2, mitad), (W // 2, H), (255, 255, 0), 2)

            # Centros carril
            if cx_left is not None:
                cv2.circle(img_draw, (cx_left, mitad + 40), 8, (0, 255, 255), -1)
            if cx_right is not None:
                cv2.circle(img_draw, (cx_right, mitad + 40), 8, (0, 255, 255), -1)
            if lane_center is not None:
                cv2.circle(img_draw, (lane_center, mitad + 80), 10, (0, 255, 0), -1)

            # Estado carril
            cv2.putText(img_draw, f"2 lineas: {ve_2_lineas}  L:{area_left} R:{area_right}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            # Timer visible
            if now < stop_hold_until:
                restante = stop_hold_until - now
                cv2.putText(img_draw, f"STOP HOLD: {restante:.1f}s",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 255), 3)
            else:
                cv2.putText(img_draw, f"VEL: {int(velocidad_obj)}  DIR: {int(direccion_obj)}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            # Vista partida (arriba imagen / abajo máscara)
            top = img_draw[:mitad, :, :]
            bottom_mask_bgr = cv2.cvtColor(mask_azul, cv2.COLOR_GRAY2BGR)
            vista = np.vstack([top, bottom_mask_bgr])

            cv2.imshow("Autonomo Carril Azul + STOP", vista)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                ejecutando = False
                break

    finally:
        ejecutando = False
        pipeline.stop()
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

direccion_obj = NEUTRO
                pi.write(LED_ROJO, 1)

            # Aplicar salidas
            velocidad_actual = int(velocidad_obj)
            direccion_actual = int(direccion_obj)

            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)

            # =====================================================
            # 4) VISUAL: arriba imagen, abajo máscara azul (BN)
            # =====================================================
            img_draw = img.copy()

            # Dibujo STOP
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE and memoria_stop["box"] is not None:
                x, y, w, h = memoria_stop["box"]
                c = (0, 255, 0) if hallado_stop else (0, 255, 255)
                cv2.drawContours(img_draw, [memoria_stop["puntos"]], 0, c, 3)
                cv2.putText(img_draw, f"STOP: {memoria_stop['distancia']:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

            # Referencia de centro
            cv2.line(img_draw, (W // 2, mitad), (W // 2, H), (255, 255, 0), 2)
            if cx is not None:
                cv2.circle(img_draw, (cx, mitad + 40), 8, (0, 255, 255), -1)

            cv2.putText(
                img_draw,
                f"carril_ok={carril_ok} area={area_azul} vel={velocidad_actual} dir={direccion_actual}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            top = img_draw[:mitad, :, :]
            bottom_mask_bgr = cv2.cvtColor(mask_azul, cv2.COLOR_GRAY2BGR)
            vista = np.vstack([top, bottom_mask_bgr])

            cv2.imshow("Control Inteligente Pi4B (STOP + Carril Azul)", vista)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                ejecutando = False
                break

    finally:
        ejecutando = False
        pipeline.stop()
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

