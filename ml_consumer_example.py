"""
Cliente de Telemetria e Otimização IA do RoboSeguidor
Integra-se com o ESP32-S3 para capturar dados e aplicar machine learning
"""

import requests
import json
import pandas as pd
import time
from datetime import datetime
import numpy as np
from collections import deque

class RoboSeguidor:
    def __init__(self, ip="192.168.1.1", port=80):
        self.ip = ip
        self.port = port
        self.base_url = f"http://{ip}:{port}"
        self.dados_cache = []
        self.conectado = False
        
    def verificar_conexao(self):
        """Verifica se o robô está acessível"""
        try:
            response = requests.get(f"{self.base_url}/", timeout=2)
            self.conectado = response.status_code == 200
            return self.conectado
        except:
            self.conectado = False
            return False
    
    def obter_analise_ia(self):
        """Obtém análise de IA e recomendações do ESP32"""
        try:
            print(f"[INFO] Requisitando análise IA...")
            response = requests.get(f"{self.base_url}/tune", timeout=5)
            response.raise_for_status()
            dados = response.json()
            
            print("\n[IA ANALYSIS] " + "="*60)
            if 'metrics' in dados:
                print(f"Erro Médio: {dados['metrics']['avgError']}")
                print(f"Erro Máximo: {dados['metrics']['maxError']}")
                print(f"Instabilidades: {dados['metrics']['instabilityCount']}")
                print(f"Aceleração Y: {dados['metrics']['accelY']['avg']:.3f}g")
            
            if 'aiRecommendation' in dados:
                rec = dados['aiRecommendation']
                print(f"\n[RECOMENDAÇÃO]")
                print(f"  Kp: {rec['kp']:.6f}")
                print(f"  Ki: {rec['ki']:.8f}")
                print(f"  Kd: {rec['kd']:.6f}")
                print(f"  Confiança: {rec['confidence']*100:.0f}%")
                print(f"  Motivo: {rec['reasoning']}")
            
            print("="*60 + "\n")
            return dados
            
        except Exception as e:
            print(f"[ERRO] Falha ao obter análise: {e}")
            return None
    
    def aplicar_pid(self, kp, kd, ki=None):
        """Aplica novos valores de PID no robô"""
        try:
            params = {'kp': kp, 'kd': kd}
            if ki is not None:
                params['ki'] = ki
            
            print(f"[INFO] Aplicando PID: Kp={kp:.6f}, Kd={kd:.6f}")
            response = requests.get(f"{self.base_url}/aplicar-pid", params=params, timeout=3)
            resultado = response.json()
            
            if resultado.get('status') == 'OK':
                print(f"[✓] PID aplicado com sucesso!")
                return True
            else:
                print(f"[✗] Erro: {resultado.get('error')}")
                return False
        except Exception as e:
            print(f"[ERRO] Falha ao aplicar PID: {e}")
            return False
    
    def obter_dados_csv(self):
        """Baixa histórico completo em CSV"""
        try:
            print(f"[INFO] Baixando CSV...")
            response = requests.get(f"{self.base_url}/dados.csv", timeout=10)
            response.raise_for_status()
            
            filename = f"telemetria_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, 'w') as f:
                f.write(response.text)
            print(f"[✓] CSV salvo: {filename}")
            return filename
        except Exception as e:
            print(f"[ERRO] {e}")
            return None
    
    def analisar_dados_localmente(self, arquivo_csv=None):
        """Analisa dados com pandas e scikit-learn"""
        try:
            if arquivo_csv:
                df = pd.read_csv(arquivo_csv)
            else:
                # Baixar dados primeiro
                arquivo = self.obter_dados_csv()
                if not arquivo:
                    return None
                df = pd.read_csv(arquivo)
            
            print(f"\n[ANÁLISE LOCAL] {len(df)} pontos de dados")
            print(f"Coluna: {df.columns.tolist()}")
            
            # Estatísticas básicas
            print(f"\nErro (posição):")
            print(f"  Médio: {df['erro'].mean():.0f}")
            print(f"  Std: {df['erro'].std():.0f}")
            print(f"  Min: {df['erro'].min()}")
            print(f"  Max: {df['erro'].max()}")
            
            print(f"\nCorreção PID:")
            print(f"  Médio: {df['correcao'].mean():.2f}")
            print(f"  Max: {df['correcao'].max():.2f}")
            
            # Detectar instabilidades
            deltas = df['erro'].diff().abs()
            instabel = (deltas > 500).sum()
            print(f"\nInstabilidades (Δ>500): {instabel}")
            
            return df
            
        except Exception as e:
            print(f"[ERRO] {e}")
            return None
    
    def otimizar_com_ml(self):
        """Pipeline completo de otimização"""
        print("[START] Iniciando otimização com IA...")
        
        if not self.verificar_conexao():
            print("[ERRO] Robô não está acessível em " + self.base_url)
            return
        
        # Esperar um tempo para coletar dados
        print("[INFO] Coletando dados por 10 segundos...")
        time.sleep(10)
        
        # Obter análise IA
        analise = self.obter_analise_ia()
        
        if analise and 'aiRecommendation' in analise:
            rec = analise['aiRecommendation']
            
            # Aplicar recomendação
            if input("\nDeseja aplicar a recomendação? (S/n): ").lower() != 'n':
                self.aplicar_pid(rec['kp'], rec['kd'], rec.get('ki', 0.00001))
                
                # Novo ciclo depois de aplicar
                print("\n[INFO] Aguardando dados com novo PID...")
                time.sleep(10)
                
                # Obter nova análise
                nova_analise = self.obter_analise_ia()
                print("[INFO] Ciclo de otimização concluído!")
    
    def stream_ao_vivo(self, duracao_segundos=30):
        """Stream contínuo de dados em tempo real"""
        print(f"[INFO] Iniciando stream por {duracao_segundos}s...")
        tempo_inicio = time.time()
        contador = 0
        
        try:
            while time.time() - tempo_inicio < duracao_segundos:
                try:
                    response = requests.get(f"{self.base_url}/dados.json", timeout=1)
                    dados = response.json()
                    
                    if dados.get('t'):
                        contador += 1
                        pos = dados.get('pos', 0)
                        err = dados.get('err', 0)
                        pwm_e = dados.get('pwmE', 0)
                        pwm_d = dados.get('pwmD', 0)
                        
                        print(f"[{contador}] Pos:{pos:4d} Err:{err:5d} PWM-E:{pwm_e:3d} PWM-D:{pwm_d:3d}")
                        self.dados_cache.append(dados)
                        
                except Exception as e:
                    pass
                
                time.sleep(0.1)
            
            print(f"\n[✓] Capturados {contador} pontos")
            return self.dados_cache
            
        except KeyboardInterrupt:
            print(f"\n[✓] Stream interrompido - {contador} pontos capturados")
            return self.dados_cache
    
    def resetar_buffer(self):
        """Reseta o buffer do robô"""
        try:
            response = requests.get(f"{self.base_url}/reset", timeout=2)
            print("[✓] Buffer resetado")
            return True
        except Exception as e:
            print(f"[ERRO] {e}")
            return False


# Exemplo de uso
if __name__ == "__main__":
    print("""
    ╔════════════════════════════════════════╗
    ║   RoboSeguidor - Cliente IA v2.0       ║
    ║   Otimização de PID com Machine Learning║
    ╚════════════════════════════════════════╝
    """)
    
    robo = RoboSeguidor()
    
    while True:
        print("\n[MENU]")
        print("1) Verificar conexão")
        print("2) Obter análise IA")
        print("3) Aplicar PID manualmente")
        print("4) Stream ao vivo (30s)")
        print("5) Baixar CSV")
        print("6) Otimizar completo (coleta + IA + aplicar)")
        print("7) Sair")
        
        opcao = input("\nEscolha: ").strip()
        
        if opcao == '1':
            if robo.verificar_conexao():
                print("[✓] Conectado!")
            else:
                print("[✗] Desconectado")
        
        elif opcao == '2':
            robo.obter_analise_ia()
        
        elif opcao == '3':
            kp = float(input("Kp: "))
            kd = float(input("Kd: "))
            ki = float(input("Ki (opcional): ") or "0.00001")
            robo.aplicar_pid(kp, kd, ki)
        
        elif opcao == '4':
            robo.stream_ao_vivo(30)
        
        elif opcao == '5':
            robo.obter_dados_csv()
        
        elif opcao == '6':
            robo.otimizar_com_ml()
        
        elif opcao == '7':
            break
    
    print("\n[BYE] Até logo!")

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
