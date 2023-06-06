import serial
import simpleaudio as sa
import numpy as np

# Configure the serial port
ser = serial.Serial('/dev/tty.usbmodem144103', baudrate=115200)

# Utwórz prosty dźwięk sinusoidalny
frequency = 50  # Domyślna częstotliwość dźwięku w Hz (tutaj: A4)

# Odtwarzaj dźwięki na podstawie danych z portu szeregowego
while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').rstrip()
        values = data.split(',')

        if len(values) >= 1:
            try:
                new_frequency = int(values[0])  # Odczytaj pierwszą wartość przed przecinkiem i konwertuj na liczbę całkowitą

                # Sprawdź, czy wartość czestotliwości mieści się w zakresie od -1500 do 1500
                if -20000 <= new_frequency <= 20000:
                    frequency = new_frequency 
                    print(frequency) # Zaktualizuj częstotliwość dźwięku

                else:
                    print("Nieprawidłowa wartość czestotliwości.")
            except ValueError:
                print("Nieprawidłowy format danych.")

    # Generuj dane dźwiękowe
    t = np.linspace(0, 0.1, int(0.1 * 44100), False)  # Generuj czas próbkowania (1 sekunda)
    audio_data = np.sin(2 * np.pi * frequency * t)  # Generuj dane dźwiękowe

    # Odtwórz dźwięk
    play_obj = sa.play_buffer((audio_data * 32767).astype(np.int16), 1, 2, 44100)
    play_obj.wait_done()
