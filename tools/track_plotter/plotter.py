import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import platform
import math

# --- CONFIGURAÇÃO ---
if platform.system() == 'Linux':
    # Tente ttyACM1 ou ttyACM0
    SERIAL_PORT = '/dev/ttyACM1'
else:
    SERIAL_PORT = 'COM4'

BAUD_RATE = 38400

# Listas para guardar os pontos em METROS
x_meters = []
y_meters = []

# Variáveis para guardar o ponto de origem (Zero)
origin_lat = None
origin_lon = None

# Configuração do Gráfico
fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-', linewidth=2, label='Trajeto (m)') # Linha azul
point, = ax.plot([], [], 'ro', label='Atual')                   # Ponto vermelho

ax.set_title(f"Trajeto em Metros (Relativo ao Início)")
ax.set_xlabel("Distância Leste-Oeste (metros)")
ax.set_ylabel("Distância Norte-Sul (metros)")
ax.grid(True)
ax.legend()
ax.axis('equal') # IMPORTANTE: Garante que 1 metro em X seja igual a 1 metro em Y

# Fatores de conversão (aproximação para pequenas distâncias)
METERS_PER_DEG_LAT = 111132.954
meters_per_deg_lon = 0 # Será calculado baseado na latitude

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Conectado na {SERIAL_PORT}!")
    print("Aguardando movimento para definir o ponto ZERO...")
except Exception as e:
    print(f"Erro ao conectar: {e}")
    exit()

def latlon_to_meters(lat, lon, org_lat, org_lon):
    # Diferença em graus
    d_lat = lat - org_lat
    d_lon = lon - org_lon
    
    # Converte para metros
    # Y = Variação de Latitude * Metros por Grau
    y = d_lat * METERS_PER_DEG_LAT
    
    # X = Variação de Longitude * Metros por Grau (depende da latitude atual)
    # Calculamos o fator de longitude baseado na latitude da origem
    x = d_lon * (METERS_PER_DEG_LAT * math.cos(math.radians(org_lat)))
    
    return x, y

def update(frame):
    global origin_lat, origin_lon, meters_per_deg_lon

    while ser.in_waiting:
        try:
            line_str = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if ',' in line_str:
                parts = line_str.split(',')
                if len(parts) == 2:
                    lat = float(parts[0])
                    lon = float(parts[1])
                    
                    if lat != 0 and lon != 0:
                        # Se for o primeiro ponto válido, define como ORIGEM (0,0)
                        if origin_lat is None:
                            origin_lat = lat
                            origin_lon = lon
                            print(f"Origem definida em: {origin_lat}, {origin_lon}")
                        
                        # Converte para metros relativos à origem
                        x, y = latlon_to_meters(lat, lon, origin_lat, origin_lon)
                        
                        x_meters.append(x)
                        y_meters.append(y)
                        
                        # Limita histórico
                        if len(x_meters) > 2000:
                            x_meters.pop(0)
                            y_meters.pop(0)

        except ValueError:
            pass
        except Exception:
            pass

    if x_meters:
        line.set_data(x_meters, y_meters)
        point.set_data([x_meters[-1]], [y_meters[-1]])
        ax.relim()
        ax.autoscale_view()
    
    return line, point

ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()

if ser.is_open:
    ser.close()
