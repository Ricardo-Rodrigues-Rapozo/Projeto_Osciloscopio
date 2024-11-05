import serial
import time
import numpy as np
import matplotlib.pyplot as plt

#Função para enviar sinal contínuo para o microcontrolador
def send_sync_signal():
    sync_byte = bytes([1])  # Envia o byte 0xAA
    ser.write(sync_byte)
    print("Sinal de sincronização enviado : ",sync_byte)
    time.sleep(0.5)  # Espera 1 segundo antes de enviar o próximo sinal

# Configura a porta serial  
ser = serial.Serial('COM3', 115200, timeout=None)
ser.flush() ## limpa dados pendentes na porta serial 
time.sleep(1)  # Espera a conexão ser estabelecida

# Cria um vetor de zeros para armazenar 1000 valores uint16
BUFFER_SIZE = 5000
valor_inteiro = 1
data = []
vect = np.zeros(BUFFER_SIZE, dtype=np.uint16)



while True:    
    send_sync_signal()

    if ser.in_waiting > (4*BUFFER_SIZE-1):
        data = np.frombuffer(ser.read(BUFFER_SIZE*4),dtype=np.uint16)*3.3/4095
        # Calcula a FFT do vetor
        #fft_result = np.fft.fft(data[BUFFER_SIZE*0.01:BUFFER_SIZE*0,9])
        # Define o intervalo para a FFT (de 1% a 90% dos dados)
        data_slice = data[int(BUFFER_SIZE * 0.01):int(BUFFER_SIZE * 0.90)]
        fft_result = np.fft.fft(data_slice)

        
        # Frequências associadas aos componentes da FFT
        frequencies = np.fft.fftfreq(len(data_slice), d=1/40000)  # 40000 Hz = Taxa de amostragem

        # Pega o valor absoluto (magnitude) da FFT
        fft_magnitude = np.abs(fft_result)

        # Filtra apenas as frequências de interesse (por exemplo, até 500 Hz)
        idx = np.where(frequencies > 0)[0]  # Considera apenas as frequências positivas
        freq_filtered = frequencies[idx]
        fft_magnitude_filtered = fft_magnitude[idx]

        # Cria subplots (2 gráficos, 1 coluna)
        plt.subplot(2, 1, 1)  # Primeiro gráfico (sinal no tempo)
        plt.plot(data_slice)
        plt.title('Sinal Serial (dados de entrada)')

        plt.subplot(2, 1, 2)  # Segundo gráfico (FFT)
        plt.plot(freq_filtered, fft_magnitude_filtered)  # Mostra até 500 Hz
        plt.title('FFT do sinal ')

        # Mostra os gráficos
        plt.tight_layout()  # Ajusta o layout para evitar sobreposição
        plt.show()
