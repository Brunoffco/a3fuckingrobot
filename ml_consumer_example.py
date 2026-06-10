"""
Exemplo de consumo de dados de telemetria do RoboSeguidor via WiFi
Para usar Machine Learning em modelos locais (TensorFlow, Scikit-Learn, etc)
"""

import requests
import json
import pandas as pd
import time
from datetime import datetime

class RoboSeguidor:
    def __init__(self, ip="192.168.1.1"):
        self.ip = ip
        self.base_url = f"http://{ip}"
        self.dados_cache = []
    
    def conectar_telemetria_sse(self):
        """Conecta ao stream SSE de telemetria em tempo real"""
        print(f"[INFO] Conectando ao {self.base_url}/telemetria")
        try:
            response = requests.get(f"{self.base_url}/telemetria", stream=True)
            for linha in response.iter_lines():
                if linha:
                    try:
                        # Remove prefixo "data: " do SSE
                        if linha.startswith(b'data: '):
                            json_str = linha[6:].decode('utf-8')
                            dados = json.loads(json_str)
                            self.dados_cache.append(dados)
                            print(f"[LIVE] Pos: {dados['pos']} | Err: {dados['err']} | PWM: {dados['pwmE']}/{dados['pwmD']}")
                    except Exception as e:
                        pass
        except Exception as e:
            print(f"[ERRO] Falha de conexão: {e}")
    
    def baixar_csv(self):
        """Baixa histórico de dados em CSV"""
        print(f"[INFO] Baixando CSV...")
        try:
            response = requests.get(f"{self.base_url}/dados.csv")
            with open(f"telemetria_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", 'w') as f:
                f.write(response.text)
            print("[OK] CSV baixado")
        except Exception as e:
            print(f"[ERRO] {e}")
    
    def baixar_ml_json(self):
        """Baixa dados normalizados para ML (0-1)"""
        print(f"[INFO] Baixando dados ML (normalizados)...")
        try:
            response = requests.get(f"{self.base_url}/ml.json")
            dados = response.json()
            print(f"[OK] {len(dados)} amostras recebidas")
            return dados
        except Exception as e:
            print(f"[ERRO] {e}")
            return []
    
    def processar_para_ml(self):
        """Converte dados para DataFrame do Pandas para ML"""
        dados = self.baixar_ml_json()
        if not dados:
            return None
        
        df = pd.DataFrame(dados)
        print(f"\nDataFrame com {len(df)} linhas:")
        print(df.head())
        return df
    
    def treinar_modelo_pid_correcao(self):
        """
        Exemplo: Treina modelo para prever melhor correção PID
        baseado em posição sensor e erro
        """
        from sklearn.ensemble import RandomForestRegressor
        from sklearn.model_selection import train_test_split
        
        df = self.processar_para_ml()
        if df is None or len(df) < 10:
            print("[AVISO] Dados insuficientes")
            return None
        
        # Features: posição normalizada, erro normalizado, KP, KI, KD
        X = df[['pos', 'e', 'kp', 'ki', 'kd']]
        # Target: correção predita
        y = df['c']
        
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
        
        modelo = RandomForestRegressor(n_estimators=10, max_depth=5)
        modelo.fit(X_train, y_train)
        
        score = modelo.score(X_test, y_test)
        print(f"[ML] Modelo R² score: {score:.3f}")
        
        # Salva modelo em formato ONNX para ESP32 (posterior)
        # import skl2onnx
        # inicial_types = [('float_input', FloatTensorType([-1, 5]))]
        # onyx_model = convert_sklearn(modelo, initial_types=inicial_types)
        
        return modelo


# Exemplo de uso
if __name__ == "__main__":
    robo = RoboSeguidor(ip="192.168.1.1")
    
    # Opção 1: Monitorar telemetria em tempo real
    # robo.conectar_telemetria_sse()
    
    # Opção 2: Baixar dados após corrida
    # robo.baixar_csv()
    
    # Opção 3: Processar dados para ML
    df = robo.processar_para_ml()
    
    # Opção 4: Treinar modelo
    # modelo = robo.treinar_modelo_pid_correcao()
