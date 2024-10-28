import serial
import time
import numpy as np
import matplotlib.pyplot as plt

#Função para enviar sinal contínuo para o microcontrolador
def send_sync_signal():
    sync_byte = bytes([1])  # Envia o byte 0xAA
    ser.write(sync_byte)
    print("Sinal de sincronização enviado : ",sync_byte)
    time.sleep(1)  # Espera 1 segundo antes de enviar o próximo sinal

# Configura a porta serial
ser = serial.Serial('COM3', 115200, timeout=None)
ser.flush() ## limpa dados pendentes na porta serial 
time.sleep(1)  # Espera a conexão ser estabelecida

# Cria um vetor de zeros para armazenar 1000 valores uint16
vect = np.zeros(1000, dtype=np.uint16)
valor_inteiro = 1
data = []
BUFFER_SIZE = 1000

while True:    
    send_sync_signal()
    if ser.in_waiting > (4*BUFFER_SIZE-1):
        data = np.frombuffer(ser.read(BUFFER_SIZE*4),dtype=np.int32)
        # Calcula a FFT do vetor
        fft_result = np.fft.fft(data)
        # Pega o valor absoluto (magnitude) da FFT
        fft_magnitude = np.abs(fft_result)
        # Cria subplots (2 linhas, 1 coluna)
        plt.subplot(2, 1, 1)  # Primeiro gráfico
        plt.plot(data)
        plt.title('Dados Serial')

        plt.subplot(2, 1, 2)  # Segundo gráfico
        plt.plot(fft_magnitude)
        plt.title('FFT dos dados ')

        # Mostra os gráficos
        plt.tight_layout()  # Ajusta para evitar sobreposição
        plt.show()
        