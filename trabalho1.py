import serial
import time
import numpy as np
import matplotlib.pyplot as plt

#Função para enviar sinal contínuo para o microcontrolador
def send_sync_signal():
    sync_byte = bytes([1])  # Envia o byte 0xAA
    ser.write(sync_byte)
    print("Sinal de sincronização enviado : ",sync_byte)
    time.sleep(0.1)  # Espera 1 segundo antes de enviar o próximo sinal

# Configura a porta serial  
ser = serial.Serial('COM5', 115200, timeout=None)
ser.flush() ## limpa dados pendentes na porta serial 
time.sleep(1)  # Espera a conexão ser estabelecida

# Cria um vetor de zeros para armazenar 1000 valores uint16
BUFFER_SIZE = 200
FS = 40000
valor_inteiro = 1
data = []
vect = np.zeros(BUFFER_SIZE, dtype=np.uint16)



while True:    
    #ser.flush() ## limpa dados pendentes na porta serial 
    send_sync_signal()

    ser.flush()
    while ser.in_waiting > (2*BUFFER_SIZE-1):

        data = np.frombuffer(ser.read(BUFFER_SIZE*2),dtype=np.uint16)*3.3/4095
        # Calcula a FFT do vetor
        #fft_result = np.fft.fft(data[BUFFER_SIZE*0.01:BUFFER_SIZE*0,9])
        # Define o intervalo para a FFT (de 1% a 90% dos dados)
        # Plota o sinal recebido no tempo (seu primeiro gráfico)
        plt.subplot(2, 1, 1)
        plt.plot(data)
        plt.title('Sinal Serial (dados de entrada)')

        # Calcula a FFT do sinal completo
        fft_result = np.fft.fft(data)
        frequencies = np.fft.fftfreq(len(data), d=1 / FS)

        # Pega o valor absoluto (magnitude) da FFT
        fft_magnitude = np.abs(fft_result)

        # Filtra as frequências positivas
        idx = np.where(frequencies >= 0)
        freq_filtered = frequencies[idx]
        fft_magnitude_filtered = fft_magnitude[idx]

        # Plota o resultado da FFT (seu segundo gráfico)
        plt.subplot(2, 1, 2)
        plt.plot(freq_filtered, fft_magnitude_filtered)
        plt.title('FFT do sinal')

        # Ajusta o layout para evitar sobreposição
        plt.tight_layout()
        plt.show()
