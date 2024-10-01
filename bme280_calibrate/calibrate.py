import numpy as np

def get_calibration_coefficients(control_temperatures, actual_temperatures):
    # Рассчитываем ошибки
    errors = control_temperatures - actual_temperatures
    
    # Находим коэффициенты полинома 3-й степени для аппроксимации ошибки
    coefficients = np.polyfit(control_temperatures, errors, 3)
    
    # Округляем коэффициенты до 6 знаков
    coefficients = np.round(coefficients, 6)
    
    # Форматируем вывод для C++
    print(f"#define COEFF_A {coefficients[0]}")
    print(f"#define COEFF_B {coefficients[1]}")
    print(f"#define COEFF_C {coefficients[2]}")
    print(f"#define COEFF_D {coefficients[3]}")
    # print(f"#define COEFF_E {coefficients[4]}")
    
    return coefficients

def correct_temperature(raw_temperature, coefficients):
    a, b, c, d = coefficients
    correction = a * raw_temperature**3 + b * raw_temperature**2 + c * raw_temperature + d
    corrected_temperature = raw_temperature + correction
    return np.round(corrected_temperature, 4)

# Контрольные температуры и фактические показания
# Начальная температура должна быть как можно ближе к типовой комнатной
control_temperatures = np.array([10, 63, 87, 101])  # Температуры поверенного датчика
actual_temperatures = np.array([10, 55, 75, 85])  # Температуры BME280

# Коэффициенты калибровки
coefficients = get_calibration_coefficients(control_temperatures, actual_temperatures)

# Эмуляция корректировки для температур 10°C, 25°C и контрольных температур
test_temperatures = np.array([10, 25, 45, 50, 75, 90])

# Вывод скорректированных значений для эмулированных температур
for temp in test_temperatures:
    corrected_temp = correct_temperature(temp, coefficients)
    print(f"Эмулированная исходная температура: {temp}°C, Корректированная температура: {corrected_temp}°C")
