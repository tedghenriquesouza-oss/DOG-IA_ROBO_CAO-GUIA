# -*- coding: utf-8 -*-
"""
DOG-IA V2 — Simulador Urbano de Cão-Guia Autônomo
AC-4 — Entrega Final

Evolução da V1:
  Marco 1: semáforo com fases All-Red (dt em segundos), veículos com cinemática
           (ACEL/DESACEL, distância de seguimento, linha de retenção), veículos
           infratores e barreira física de meio-fio (aplicar_barreira).
  Marco 2: arquitetura híbrida — planejador global A* em grade 10x10 px com
           C-Space assimétrico (solo 30, edificação 22, suspenso 38), custo 0.45
           no piso tátil, destinos 1-4 e replanejamento após 3,5 s em desvio local.
  Marco 3: LiDAR por interseção analítica (rect.clipline), ruído gaussiano
           (sigma 1,5 px), dropout de 2 %, filtro de mediana temporal (deque 3)
           e predição de colisão por zona varrida (swept volume, dt = 1,0 s).
  Marco 4: tutor como entidade independente ligada pelo arnês (GUIA = 42 px),
           redução de velocidade por tensão, alertas sonoros estéreo (pan -1..+1),
           métricas da viagem e execução headless (--benchmark).

Uso:
  python dog_ia_v2.py                          # simulação interativa
  python dog_ia_v2.py --benchmark 20 --seed 1  # avaliação experimental headless
  python dog_ia_v2.py --viagem 1 --seed 1      # uma viagem headless (Casa -> Mercado) com log de estados
"""
import argparse
import heapq
import math
import os
import random
import sys
from array import array
from collections import deque

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

# ==================== CONFIGURAÇÕES GERAIS ====================
LARGURA, ALTURA = 1050, 680
FPS = 60
DT_BENCHMARK = 1.0 / FPS
AREA_MAPA_LARGURA = LARGURA - 260

# Geometria da avenida e da faixa de pedestres
RUA_Y = 250
RUA_ALTURA = 170
FAIXA_X = 580
FAIXA_LARGURA = 95
FAIXA_X0 = FAIXA_X + 8                    # limites que o centro do robô pode usar na faixa
FAIXA_X1 = FAIXA_X + FAIXA_LARGURA - 8
MARGEM_MEIO_FIO = 8
BANDA_RUA_Y0 = RUA_Y - MARGEM_MEIO_FIO    # centro do robô nesta banda = "desceu o meio-fio"
BANDA_RUA_Y1 = RUA_Y + RUA_ALTURA + MARGEM_MEIO_FIO

# Paleta de Cores Urbanas e Assistivas
BRANCO = (245, 245, 245)
PRETO = (20, 22, 28)
ASFALTO = (42, 45, 52)
CALCADA_CLARA = (130, 134, 142)
MEIO_FIO = (175, 180, 188)
PISO_TATIL = (235, 190, 20)
GRAMA = (46, 115, 65)
CINZA_TEXTO = (170, 176, 186)

# Cores do Cão-Robô e Sensores
CORPO_ROBO = (30, 136, 229)
DETALHE_ROBO = (15, 76, 129)
CABECA_ROBO = (230, 236, 245)
ARNES_GUIA = (255, 215, 0)
VERDE_LIVRE = (46, 204, 113)
LARANJA_ALERTA = (255, 140, 0)
VERMELHO_PERIGO = (225, 55, 55)
ROXO_SUSPENSO = (155, 89, 182)
CIANO_ROTA = (0, 200, 255)
AMARELO_SINAL = (250, 200, 40)
VERMELHO_INFRATOR = (200, 30, 30)
CORES_CARROS = [(210, 210, 215), (60, 90, 160), (40, 40, 45), (150, 155, 165),
                (90, 140, 90), (200, 160, 60), (110, 70, 130)]

# ---------- Marco 1: semáforo ----------
FASES_SEMAFORO = [
    ("VERDE", 9.0),         # verde para veículos
    ("AMARELO", 3.0),
    ("TODOS_VERM_1", 1.5),  # All-Red de segurança
    ("PEDESTRE", 8.0),      # boneco verde para pedestres
    ("TODOS_VERM_2", 1.5),  # All-Red de segurança
]
CICLO_SEMAFORO = sum(d for _, d in FASES_SEMAFORO)
FASES_VERMELHO_VEICULO = ("TODOS_VERM_1", "PEDESTRE", "TODOS_VERM_2")

# ---------- Marco 1: veículos ----------
ACEL = 110.0              # px/s²
DESACEL = 260.0           # px/s² (frenagem de serviço)
DESACEL_EMERG = 430.0     # px/s² (frenagem forte por pedestre à frente)
DIST_SEGUIMENTO = 16.0    # px de folga mínima até o veículo da frente
TEMPO_HEADWAY = 0.55      # s de distância temporal de seguimento
VEIC_W, VEIC_H = 46, 24
FAIXA_SUP_Y = RUA_Y + 42                 # pista superior: sentido oeste (<-)
FAIXA_INF_Y = RUA_Y + RUA_ALTURA - 42    # pista inferior: sentido leste (->)
LINHA_RETENCAO_LESTE = FAIXA_X - 14                  # carros indo para a direita param aqui
LINHA_RETENCAO_OESTE = FAIXA_X + FAIXA_LARGURA + 14  # carros indo para a esquerda param aqui
PROB_INFRATOR = 0.10
DETECCAO_PEDESTRE_NORMAL = 170.0
DETECCAO_PEDESTRE_INFRATOR = 95.0

# ---------- Robô / tutor ----------
VELOCIDADE = 156.0        # px/s
VEL_ANGULAR = 4.0         # rad/s
ACEL_ROBO = 320.0
DESACEL_ROBO = 650.0
GUIA = 42.0               # comprimento do arnês (px)
FOLGA_TENSAO = 10.0       # px de tensão tolerada antes de reduzir a velocidade
VEL_TUTOR_MAX = 175.0
GANHO_TUTOR = 16.0
RAIO_CHEGADA = 18.0
LOOKAHEAD = 26.0
LIMITE_TRAVESSIA = (RUA_ALTURA + 24 + GUIA) / VELOCIDADE * 1.35 + 0.4

# ---------- Marco 2: planejamento ----------
CELULA = 10
INFLA_SOLO = 30
INFLA_EDIF = 22
INFLA_SUSP = 38
CUSTO_PISO_TATIL = 0.45
TEMPO_MAX_DESVIO = 3.5
DIST_DESVIO_DESCONHECIDO = 58.0   # obstáculo fora do mapa do planejador
DIST_DESVIO_CONHECIDO = 25.0      # obstáculo já inflado no C-Space
DIST_LATERAL = 27.0
SQRT2 = math.sqrt(2.0)

# ---------- Marco 3: sensoriamento ----------
LIDAR_SIGMA = 1.5
LIDAR_DROPOUT = 0.02
JANELA_MEDIANA = 3
HORIZONTE_PREDICAO = 1.0
GAP_QUASE_COLISAO = 12.0

# Estados da máquina de estados
PARADO = "PARADO"
PLANEJANDO = "PLANEJANDO"
NAVEGANDO = "NAVEGANDO"
DESVIO_LOCAL = "DESVIO_LOCAL"
AGUARDANDO_SINAL = "AGUARDANDO_SINAL"
ATRAVESSANDO = "ATRAVESSANDO"
FREIO_EMERGENCIA = "FREIO_EMERGENCIA"
DESTINO_ALCANCADO = "DESTINO_ALCANCADO"
SEM_ROTA = "SEM_ROTA"
MANUAL = "MANUAL"

COR_ESTADO = {
    PARADO: CINZA_TEXTO, PLANEJANDO: CIANO_ROTA, NAVEGANDO: VERDE_LIVRE,
    DESVIO_LOCAL: VERMELHO_PERIGO, AGUARDANDO_SINAL: LARANJA_ALERTA,
    ATRAVESSANDO: CIANO_ROTA, FREIO_EMERGENCIA: VERMELHO_PERIGO,
    DESTINO_ALCANCADO: VERDE_LIVRE, SEM_ROTA: VERMELHO_PERIGO, MANUAL: BRANCO,
}

CASA = (95, 127)
DESTINOS = {
    1: ("Mercado", (180, 543)),
    2: ("Farmácia", (735, 560)),
    3: ("Padaria", (735, 150)),
    4: ("Casa", CASA),
}

PISOS_TATEIS = [
    pygame.Rect(40, 120, 595, 14),
    pygame.Rect(621, 120, 14, 130),
    pygame.Rect(621, RUA_Y + RUA_ALTURA, 14, 130),
    pygame.Rect(150, 536, 485, 14),
]


# ==================== FUNÇÕES AUXILIARES ====================
def rotacionar_ponto(cx, cy, px, py, angulo):
    """Rotaciona um ponto (px, py) ao redor de (cx, cy) dado um ângulo em radianos."""
    s = math.sin(angulo)
    c = math.cos(angulo)
    dx = px - cx
    dy = py - cy
    return (int(cx + (dx * c - dy * s)), int(cy + (dx * s + dy * c)))


def normalizar_angulo(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def limitar(v, lo, hi):
    return max(lo, min(hi, v))


def dist_ponto_rect(x, y, rect):
    px = limitar(x, rect.left, rect.right)
    py = limitar(y, rect.top, rect.bottom)
    return math.hypot(x - px, y - py)


def circulo_rect(x, y, raio, rect):
    return dist_ponto_rect(x, y, rect) < raio


def em_banda_rua(y):
    return BANDA_RUA_Y0 <= y <= BANDA_RUA_Y1


def na_faixa_pedestre(x):
    return FAIXA_X0 <= x <= FAIXA_X1


# ==================== MARCO 1: SEMÁFORO ====================
class Semaforo:
    """Ciclo VERDE -> AMARELO -> TODOS_VERM_1 -> PEDESTRE -> TODOS_VERM_2 em tempo simulado."""

    def __init__(self, deslocamento=0.0):
        self.i = 0
        self.t = 0.0
        self.atualizar(deslocamento)

    def atualizar(self, dt):
        self.t += dt
        while self.t >= FASES_SEMAFORO[self.i][1]:
            self.t -= FASES_SEMAFORO[self.i][1]
            self.i = (self.i + 1) % len(FASES_SEMAFORO)

    @property
    def fase(self):
        return FASES_SEMAFORO[self.i][0]

    @property
    def tempo_restante(self):
        return FASES_SEMAFORO[self.i][1] - self.t


# ==================== MARCO 1: VEÍCULOS ====================
class Veiculo:
    def __init__(self, vid, sentido, x, vmax, infrator, cor):
        self.id = vid
        self.sentido = sentido            # +1 = leste (pista inferior), -1 = oeste (pista superior)
        self.x = float(x)                 # centro
        self.y = FAIXA_INF_Y if sentido == 1 else FAIXA_SUP_Y
        self.vmax = vmax
        self.v = vmax * 0.9
        self.infrator = infrator
        self.cor = cor
        self.passa_amarelo = None
        self.freando = False

    @property
    def rect(self):
        return pygame.Rect(int(self.x - VEIC_W / 2), int(self.y - VEIC_H / 2), VEIC_W, VEIC_H)

    @property
    def frente(self):
        return self.x + self.sentido * VEIC_W / 2

    @property
    def traseira(self):
        return self.x - self.sentido * VEIC_W / 2

    def zona_varrida(self, horizonte=HORIZONTE_PREDICAO):
        """Retângulo que o veículo ocupará do instante atual até t + horizonte (swept volume)."""
        atual = self.rect
        futuro = atual.move(int(self.sentido * self.v * horizonte), 0)
        return atual.union(futuro).inflate(12, 12)


# ==================== MARCO 2: PLANEJADOR A* ====================
VIZINHOS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


class PlanejadorAEstrela:
    """Grade de ocupação 10x10 px + C-Space com inflação assimétrica + A* com heurística octile."""

    def __init__(self):
        self.cols = AREA_MAPA_LARGURA // CELULA
        self.rows = ALTURA // CELULA
        self.custo = [[1.0] * self.cols for _ in range(self.rows)]
        self.inflados = []

    @staticmethod
    def margem(tipo):
        return {"solo": INFLA_SOLO, "edificacao": INFLA_EDIF, "suspenso": INFLA_SUSP}[tipo]

    def construir(self, obstaculos):
        self.inflados = []
        for o in obstaculos:
            if o["conhecido"]:
                m = self.margem(o["tipo"])
                self.inflados.append((o["rect"].inflate(2 * m, 2 * m), o["tipo"]))
        rects = [r for r, _ in self.inflados]
        for r in range(self.rows):
            cy = r * CELULA + CELULA // 2
            linha_na_rua = em_banda_rua(cy)
            for c in range(self.cols):
                cx = c * CELULA + CELULA // 2
                # Barreira de meio-fio: a pista só é transitável dentro da faixa de pedestres
                if linha_na_rua and not (FAIXA_X0 + 7 <= cx <= FAIXA_X1 - 7):
                    self.custo[r][c] = None
                    continue
                if any(rc.collidepoint(cx, cy) for rc in rects):
                    self.custo[r][c] = None
                    continue
                tatil = any(p.collidepoint(cx, cy) for p in PISOS_TATEIS)
                self.custo[r][c] = CUSTO_PISO_TATIL if tatil else 1.0

    def celula_livre_proxima(self, x, y):
        c0 = int(limitar(x // CELULA, 0, self.cols - 1))
        r0 = int(limitar(y // CELULA, 0, self.rows - 1))
        if self.custo[r0][c0] is not None:
            return (r0, c0)
        for raio in range(1, 10):
            melhor, md = None, float("inf")
            for r in range(r0 - raio, r0 + raio + 1):
                for c in range(c0 - raio, c0 + raio + 1):
                    if max(abs(r - r0), abs(c - c0)) != raio:
                        continue
                    if 0 <= r < self.rows and 0 <= c < self.cols and self.custo[r][c] is not None:
                        d = math.hypot(c * CELULA + 5 - x, r * CELULA + 5 - y)
                        if d < md:
                            melhor, md = (r, c), d
            if melhor:
                return melhor
        return None

    def planejar(self, inicio, fim):
        s = self.celula_livre_proxima(*inicio)
        g = self.celula_livre_proxima(*fim)
        if s is None or g is None:
            return None, 0.0

        def h(cel):
            # Octile multiplicada pelo menor custo de célula (0.45) => admissível
            dr = abs(cel[0] - g[0])
            dc = abs(cel[1] - g[1])
            return CUSTO_PISO_TATIL * ((dr + dc) + (SQRT2 - 2) * min(dr, dc))

        aberto = [(h(s), 0.0, s)]
        custo_g = {s: 0.0}
        pai = {s: None}
        fechado = set()
        achou = False
        while aberto:
            _, gc, cur = heapq.heappop(aberto)
            if cur in fechado:
                continue
            if cur == g:
                achou = True
                break
            fechado.add(cur)
            r, c = cur
            cc = self.custo[r][c]
            for dr, dc, passo in VIZINHOS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                    continue
                cn = self.custo[nr][nc]
                if cn is None:
                    continue
                if dr and dc and (self.custo[r][nc] is None or self.custo[nr][c] is None):
                    continue  # não corta quina de obstáculo na diagonal
                ng = gc + passo * (cc + cn) / 2.0
                if ng < custo_g.get((nr, nc), float("inf")):
                    custo_g[(nr, nc)] = ng
                    pai[(nr, nc)] = cur
                    heapq.heappush(aberto, (ng + h((nr, nc)), ng, (nr, nc)))
        if not achou:
            return None, 0.0
        celulas = []
        cur = g
        while cur is not None:
            celulas.append(cur)
            cur = pai[cur]
        celulas.reverse()
        pontos = [(c * CELULA + CELULA // 2, r * CELULA + CELULA // 2) for r, c in celulas]
        if len(pontos) > 1 and math.hypot(pontos[0][0] - inicio[0], pontos[0][1] - inicio[1]) < 6:
            pontos.pop(0)
        pontos.append((float(fim[0]), float(fim[1])))
        return pontos, custo_g[g] * CELULA


# ==================== ENTIDADES: ROBÔ E TUTOR ====================
class Tutor:
    """Pessoa com deficiência visual conduzida pelo arnês (entidade independente)."""

    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
        self.v = 0.0
        self.raio = 12


class RoboCaoGuia:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
        self.angulo = 0.0
        self.raio = 20
        self.v = 0.0
        self.alcance_lidar = 165
        self.num_raios = 19
        self.fov = math.radians(150)
        self.filtros = [deque(maxlen=JANELA_MEDIANA) for _ in range(self.num_raios)]
        self.leituras_lidar = []
        self.status_assistivo = "Selecione um destino (teclas 1 a 4)."
        self.cor_alerta = VERDE_LIVRE
        self.passo_animacao = 0.0
        self.estado = PARADO
        self.estado_antes_freio = NAVEGANDO
        self.caminho = []
        self.idx = 0
        self.travessias = []
        self.trav_idx = 0
        self.autorizado = False
        self.timer_desvio = 0.0
        self.idx_marco = 0
        self.bloqueado = False
        self.t_ultimo_conflito = -99.0
        self.veic_conflito = None
        self.tensao = 0.0

    def travessia_atual(self):
        if self.trav_idx < len(self.travessias):
            return self.travessias[self.trav_idx]
        return None

    # ---------- Marco 3: LiDAR realista ----------
    def ler_lidar(self, obstaculos, veiculos, rng):
        """Raycasting analítico com rect.clipline + ruído gaussiano + dropout + mediana temporal."""
        R = self.alcance_lidar
        area = pygame.Rect(int(self.x - R), int(self.y - R), 2 * R, 2 * R)
        candidatos = []
        for o in obstaculos:
            if area.colliderect(o["rect"]):
                candidatos.append((o["rect"], o["tipo"], o["id"], o["conhecido"]))
        for v in veiculos:
            rv = v.rect
            if area.colliderect(rv):
                candidatos.append((rv, "veiculo", -v.id, True))

        self.leituras_lidar = []
        angulo_inicial = self.angulo - self.fov / 2
        passo_angulo = self.fov / (self.num_raios - 1)
        for i in range(self.num_raios):
            ang = angulo_inicial + i * passo_angulo
            ex = self.x + math.cos(ang) * R
            ey = self.y + math.sin(ang) * R
            melhor = float(R)
            info = (None, None, True)
            for rect, tipo, oid, conhecido in candidatos:
                seg = rect.clipline(self.x, self.y, ex, ey)
                if seg:
                    (ax, ay), (bx, by) = seg
                    d = min(math.hypot(ax - self.x, ay - self.y), math.hypot(bx - self.x, by - self.y))
                    if d < melhor:
                        melhor = d
                        info = (tipo, oid, conhecido)
            # Modelo de sensor: dropout (falha => alcance máximo) e ruído gaussiano
            if rng.random() < LIDAR_DROPOUT:
                bruta, info = float(R), (None, None, True)
            elif info[0] is not None:
                bruta = limitar(melhor + rng.gauss(0.0, LIDAR_SIGMA), 0.0, float(R))
            else:
                bruta = float(R)
            self.filtros[i].append((bruta, info))
            janela = sorted(self.filtros[i], key=lambda e: e[0])
            dist_filtrada, info_f = janela[len(janela) // 2]   # mediana temporal
            self.leituras_lidar.append({
                "angulo": ang, "distancia": dist_filtrada, "bruta": bruta,
                "tipo": info_f[0], "id": info_f[1], "conhecido": info_f[2],
            })

    def desenhar(self, tela):
        # 1. Feixes do LiDAR (leituras já filtradas)
        for r in self.leituras_lidar:
            fim_x = self.x + math.cos(r["angulo"]) * r["distancia"]
            fim_y = self.y + math.sin(r["angulo"]) * r["distancia"]
            livre = r["tipo"] is None
            cor_raio = (46, 204, 113) if livre else (LARANJA_ALERTA if r["tipo"] == "veiculo" else VERMELHO_PERIGO)
            pygame.draw.line(tela, cor_raio, (self.x, self.y), (fim_x, fim_y), 1)
            if not livre:
                pygame.draw.circle(tela, BRANCO, (int(fim_x), int(fim_y)), 3)

        # 2. Patas articuladas
        balanco = math.sin(self.passo_animacao) * 5
        for ox, oy in [(10 + balanco, -14), (10 - balanco, 14), (-12 - balanco, -14), (-12 + balanco, 14)]:
            px, py = rotacionar_ponto(self.x, self.y, self.x + ox, self.y + oy, self.angulo)
            pygame.draw.circle(tela, DETALHE_ROBO, (px, py), 5)

        # 3. Tronco
        pontos_corpo = [
            rotacionar_ponto(self.x, self.y, self.x + 16, self.y - 11, self.angulo),
            rotacionar_ponto(self.x, self.y, self.x + 16, self.y + 11, self.angulo),
            rotacionar_ponto(self.x, self.y, self.x - 18, self.y + 11, self.angulo),
            rotacionar_ponto(self.x, self.y, self.x - 18, self.y - 11, self.angulo),
        ]
        pygame.draw.polygon(tela, CORPO_ROBO, pontos_corpo)
        pygame.draw.polygon(tela, BRANCO, pontos_corpo, 2)

        # 4. Cabeça, orelhas e câmera
        cab = rotacionar_ponto(self.x, self.y, self.x + 21, self.y, self.angulo)
        pygame.draw.circle(tela, CABECA_ROBO, cab, 9)
        pygame.draw.circle(tela, DETALHE_ROBO, cab, 9, 2)
        pygame.draw.circle(tela, DETALHE_ROBO, rotacionar_ponto(self.x, self.y, self.x + 19, self.y - 10, self.angulo), 4)
        pygame.draw.circle(tela, DETALHE_ROBO, rotacionar_ponto(self.x, self.y, self.x + 19, self.y + 10, self.angulo), 4)

        # 5. Torre do LiDAR
        pygame.draw.circle(tela, LARANJA_ALERTA, rotacionar_ponto(self.x, self.y, self.x + 3, self.y, self.angulo), 5)


# ==================== MARCO 4: MÉTRICAS ====================
class Metricas:
    CAMPOS = ("tempo", "distancia", "alertas_vermelhos", "freios_emergencia", "viagens_concluidas",
              "colisoes", "quase_colisoes", "replanejamentos")

    def __init__(self):
        for c in self.CAMPOS:
            setattr(self, c, 0 if c not in ("tempo", "distancia") else 0.0)

    def copia(self):
        return {c: getattr(self, c) for c in self.CAMPOS}


# ==================== MARCO 4: ÁUDIO ESTÉREO ====================
class AudioEstereo:
    """Gera tons sintéticos e toca com panorâmica (pan -1 = esquerda, +1 = direita)."""

    def __init__(self):
        self.ok = False
        self.mudo = False
        self.sons = {}
        try:
            if not pygame.mixer.get_init():
                pygame.mixer.init(22050, -16, 2, 512)
            self.freq, _, self.canais = pygame.mixer.get_init()
            pygame.mixer.set_num_channels(16)
            self.sons = {
                "laranja": self._tom([520], 0.07, 0.30),
                "vermelho": self._tom([880], 0.09, 0.40),
                "freio": self._tom([1250, 0, 1250], 0.05, 0.45),
                "travessia": self._tom([1000], 0.03, 0.25),
                "chegada": self._tom([660, 880, 990], 0.09, 0.35),
            }
            self.ok = True
        except Exception as e:  # sem placa de som, etc.
            print(f"[áudio] indisponível: {e}")

    def _tom(self, freqs, dur, vol):
        buf = array("h")
        for f in freqs:
            n = int(self.freq * dur)
            ataque = max(1, int(0.005 * self.freq))
            soltura = max(1, int(0.015 * self.freq))
            for i in range(n):
                env = min(1.0, i / ataque, (n - i) / soltura)
                s = int(32767 * vol * env * math.sin(2 * math.pi * f * i / self.freq)) if f > 0 else 0
                for _ in range(self.canais):
                    buf.append(s)
        return pygame.mixer.Sound(buffer=buf.tobytes())

    def tocar(self, tipo, pan):
        if not self.ok or self.mudo or tipo not in self.sons:
            return
        canal = self.sons[tipo].play()
        if canal:
            a = (limitar(pan, -1.0, 1.0) + 1.0) * math.pi / 4.0   # pan de potência constante
            canal.set_volume(math.cos(a), math.sin(a))


# ==================== CENÁRIO ====================
def criar_cenario():
    """Obstáculos urbanos (solo, suspensos) + limites/edificações do mapa."""
    obs = [
        {"rect": pygame.Rect(215, 130, 45, 75), "tipo": "solo", "nome": "Banca / Hidrante"},
        {"rect": pygame.Rect(612, 527, 32, 32), "tipo": "solo", "nome": "Poste de Luz"},
        {"rect": pygame.Rect(288, 523, 40, 40), "tipo": "solo", "nome": "Lixeira Urbana"},
        {"rect": pygame.Rect(350, 75, 85, 65), "tipo": "suspenso", "nome": "Copa de Árvore"},
        # Edificações / limites (canteiros e bordas do mapa)
        {"rect": pygame.Rect(0, 0, AREA_MAPA_LARGURA, 28), "tipo": "edificacao", "nome": "Fachadas Norte"},
        {"rect": pygame.Rect(0, ALTURA - 28, AREA_MAPA_LARGURA, 28), "tipo": "edificacao", "nome": "Fachadas Sul"},
        {"rect": pygame.Rect(0, 0, 10, ALTURA), "tipo": "edificacao", "nome": "Limite Oeste"},
        {"rect": pygame.Rect(AREA_MAPA_LARGURA - 10, 0, 10, ALTURA), "tipo": "edificacao", "nome": "Limite Leste"},
    ]
    for i, o in enumerate(obs):
        o["id"] = i + 1
        o["conhecido"] = True
    return obs


# ==================== SIMULAÇÃO ====================
class Simulacao:
    def __init__(self, seed=None, trace=False, imprimir_estados=False):
        self.rng = random.Random(seed)
        self.trace = trace or imprimir_estados
        self.tempo_sim = 0.0
        self.semaforo = Semaforo(self.rng.uniform(0, CICLO_SEMAFORO))
        self.obstaculos = criar_cenario()
        self.prox_id_obs = len(self.obstaculos) + 1
        self.veiculos = []
        self.prox_id_veic = 1
        self.timer_spawn = {1: self.rng.uniform(0, 2), -1: self.rng.uniform(0, 2)}
        self.robo = RoboCaoGuia(*CASA)
        self.tutor = Tutor(CASA[0] - GUIA, CASA[1])
        self.planejador = PlanejadorAEstrela()
        self.planejador.construir(self.obstaculos)
        self.metricas = Metricas()
        self.modo_autonomo = True
        self.destino_nome = None
        self.destino_pos = None
        self.fila_destinos = []
        self.viagem_ativa = False
        self.t_chegada = 0.0
        self.contatos_frame = set()
        self.contatos_ativos = {}
        self.quase_ativos = {}
        self.zona_conflito = None
        self.eventos_som = []
        self.ultimo_som = {}
        self.historico = deque(maxlen=5)
        self.log_estados = []
        # Pré-aquecimento do trânsito (10 s) para a cena não começar vazia
        for _ in range(int(10 / DT_BENCHMARK)):
            self.semaforo.atualizar(DT_BENCHMARK)
            self.atualizar_trafego(DT_BENCHMARK)

    # ---------- utilidades ----------
    def set_estado(self, novo, motivo):
        r = self.robo
        if r.estado == novo:
            return
        anterior = r.estado
        r.estado = novo
        if novo == FREIO_EMERGENCIA:
            self.metricas.freios_emergencia += 1
        if novo == DESVIO_LOCAL:
            self.metricas.alertas_vermelhos += 1
        linha = f"[t={self.tempo_sim:7.2f}s] {anterior} -> {novo} | {motivo}"
        self.historico.append((novo, motivo))
        self.log_estados.append(linha)
        if self.trace:
            print(linha)

    def emitir(self, tipo, pan, intervalo):
        if self.tempo_sim - self.ultimo_som.get(tipo, -99.0) >= intervalo:
            self.ultimo_som[tipo] = self.tempo_sim
            self.eventos_som.append((tipo, pan))

    def pan_relativo(self, x, y):
        r = self.robo
        rel = normalizar_angulo(math.atan2(y - r.y, x - r.x) - r.angulo)
        return limitar(math.sin(rel) * 1.4, -1.0, 1.0)

    def fator_tensao(self):
        r, t = self.robo, self.tutor
        r.tensao = math.hypot(r.x - t.x, r.y - t.y) - GUIA
        if r.tensao <= FOLGA_TENSAO:
            return 1.0
        return max(0.15, 1.0 - (r.tensao - FOLGA_TENSAO) / 20.0)

    # ---------- destinos e planejamento ----------
    def definir_destino(self, chave_ou_pos):
        if isinstance(chave_ou_pos, int):
            nome, pos = DESTINOS[chave_ou_pos]
        else:
            nome, pos = "Ponto livre", chave_ou_pos
        self.destino_nome = nome
        self.destino_pos = (float(pos[0]), float(pos[1]))
        self.viagem_ativa = True
        self.metricas.tempo = 0.0
        self.metricas.distancia = 0.0
        if self.modo_autonomo:
            self.planejar_rota(f"destino '{nome}' selecionado")

    def planejar_rota(self, motivo):
        r = self.robo
        self.set_estado(PLANEJANDO, motivo)
        self.planejador.construir(self.obstaculos)
        pontos, custo = self.planejador.planejar((r.x, r.y), self.destino_pos)
        if not pontos:
            r.caminho = []
            r.status_assistivo = "Sem rota segura até o destino!"
            r.cor_alerta = VERMELHO_PERIGO
            self.set_estado(SEM_ROTA, "A* não encontrou caminho no C-Space")
            return
        r.caminho = pontos
        r.idx = 0
        r.timer_desvio = 0.0
        r.travessias = []
        i = 0
        while i < len(pontos):
            if em_banda_rua(pontos[i][1]):
                j = i
                while j + 1 < len(pontos) and em_banda_rua(pontos[j + 1][1]):
                    j += 1
                r.travessias.append({"espera": max(0, i - 1), "entrada": i, "saida": j})
                i = j + 1
            else:
                i += 1
        r.trav_idx = 0
        if em_banda_rua(r.y) and r.travessias:
            r.autorizado = True
            self.set_estado(ATRAVESSANDO, f"rota A* com {len(pontos)} nós (custo {custo:.0f}); já na faixa")
        else:
            r.autorizado = False
            self.set_estado(NAVEGANDO, f"rota A* com {len(pontos)} nós (custo {custo:.0f}), "
                                       f"{len(r.travessias)} travessia(s)")

    def replanejar(self, motivo):
        # Obstáculos percebidos pelo LiDAR passam a integrar o mapa do planejador
        for o in self.obstaculos:
            o["conhecido"] = True
        self.metricas.replanejamentos += 1
        self.planejar_rota(motivo)

    def adicionar_obstaculo(self, x, y, tipo, tamanho=44):
        o = {"rect": pygame.Rect(int(x - tamanho / 2), int(y - tamanho / 2), tamanho, tamanho),
             "tipo": tipo, "nome": "Obstáculo novo", "id": self.prox_id_obs, "conhecido": False}
        self.prox_id_obs += 1
        self.obstaculos.append(o)
        return o

    # ---------- trânsito ----------
    def gerar_veiculo(self, sentido):
        x0 = -60 if sentido == 1 else AREA_MAPA_LARGURA + 60
        for v in self.veiculos:
            if v.sentido == sentido and abs(v.x - x0) < 80:
                return
        infrator = self.rng.random() < PROB_INFRATOR
        vmax = self.rng.uniform(175, 205) if infrator else self.rng.uniform(130, 175)
        cor = VERMELHO_INFRATOR if infrator else self.rng.choice(CORES_CARROS)
        self.veiculos.append(Veiculo(self.prox_id_veic, sentido, x0, vmax, infrator, cor))
        self.prox_id_veic += 1

    def atualizar_trafego(self, dt):
        for s in (1, -1):
            self.timer_spawn[s] -= dt
            if self.timer_spawn[s] <= 0:
                self.gerar_veiculo(s)
                self.timer_spawn[s] = self.rng.uniform(1.1, 3.4)

        fase = self.semaforo.fase
        r, t = self.robo, self.tutor
        entidades = [(r.x, r.y, r.raio), (t.x, t.y, t.raio)]
        for s in (1, -1):
            carros = sorted((v for v in self.veiculos if v.sentido == s), key=lambda v: v.x * s, reverse=True)
            for k, c in enumerate(carros):
                limites = [c.vmax]
                decel = DESACEL
                # Seguimento do veículo da frente
                if k > 0:
                    lider = carros[k - 1]
                    gap = (lider.traseira - c.frente) * s
                    folga = gap - DIST_SEGUIMENTO - TEMPO_HEADWAY * c.v
                    limites.append(math.sqrt(max(0.0, lider.v ** 2 + 2 * DESACEL * folga)))
                # Linha de retenção (infratores ignoram o sinal)
                if fase == "VERDE":
                    c.passa_amarelo = None
                elif c.passa_amarelo is None:
                    linha = LINHA_RETENCAO_LESTE if s == 1 else LINHA_RETENCAO_OESTE
                    d_linha = (linha - c.frente) * s
                    c.passa_amarelo = (fase == "AMARELO" and d_linha < c.v ** 2 / (2 * DESACEL) + 2) or d_linha < -2
                if not c.infrator and fase != "VERDE" and not c.passa_amarelo:
                    linha = LINHA_RETENCAO_LESTE if s == 1 else LINHA_RETENCAO_OESTE
                    d_linha = (linha - c.frente) * s
                    if d_linha > -2:
                        limites.append(math.sqrt(2 * DESACEL * max(0.0, d_linha - 1)))
                # Pedestre (robô/tutor) na pista à frente
                det = DETECCAO_PEDESTRE_INFRATOR if c.infrator else DETECCAO_PEDESTRE_NORMAL
                for ex, ey, er in entidades:
                    if abs(ey - c.y) < VEIC_H / 2 + er + 3:
                        d = (ex - c.frente) * s - er
                        if -VEIC_W < d < det:
                            limites.append(math.sqrt(2 * DESACEL_EMERG * max(0.0, d - 6)))
                            decel = DESACEL_EMERG
                v_des = min(limites)
                dv = v_des - c.v
                c.freando = dv < -1
                c.v = max(0.0, c.v + limitar(dv, -decel * dt, ACEL * dt))
                c.x += s * c.v * dt
                if k > 0:
                    lider = carros[k - 1]
                    if (lider.traseira - c.frente) * s < 2:
                        c.x = lider.traseira - s * (2 + VEIC_W / 2)
                        c.v = min(c.v, lider.v)
        self.veiculos = [v for v in self.veiculos if -90 < v.x < AREA_MAPA_LARGURA + 90]

    # ---------- Marco 3: predição por zona varrida ----------
    def prever_conflito(self, usar_corredor, ang_corredor):
        r, t = self.robo, self.tutor
        for v in self.veiculos:
            if v.v < 5:
                continue
            zona = v.zona_varrida()
            if circulo_rect(r.x, r.y, r.raio, zona):
                return v, zona, "o robô"
            if circulo_rect(t.x, t.y, t.raio, zona):
                return v, zona, "o tutor"
            if usar_corredor:
                alcance = int(VELOCIDADE * HORIZONTE_PREDICAO)
                for sd in range(15, alcance + 1, 15):
                    px = r.x + math.cos(ang_corredor) * sd
                    py = r.y + math.sin(ang_corredor) * sd
                    if circulo_rect(px, py, r.raio * 0.8, zona):
                        return v, zona, "o corredor de travessia"
        return None, None, None

    # ---------- Marco 1: barreira do meio-fio ----------
    def aplicar_barreira(self, x0, y0):
        r = self.robo
        if self.modo_autonomo:
            autorizado = r.autorizado
        else:
            autorizado = self.semaforo.fase == "PEDESTRE"
        if em_banda_rua(r.y):
            if not em_banda_rua(y0):
                # Tentando descer o meio-fio
                if not (autorizado and na_faixa_pedestre(r.x)):
                    r.y = y0
                    if not self.modo_autonomo:
                        r.status_assistivo = "BARREIRA: meio-fio! Desça só na faixa com sinal verde."
                        r.cor_alerta = LARANJA_ALERTA
            else:
                # Já está na pista: permanece dentro da faixa de pedestres
                r.x = limitar(r.x, FAIXA_X0, FAIXA_X1)

    def mover_robo(self, distancia):
        r = self.robo
        x0, y0 = r.x, r.y
        dx = math.cos(r.angulo) * distancia
        dy = math.sin(r.angulo) * distancia
        r.x += dx
        r.y += dy
        self.aplicar_barreira(x0, y0)
        r.bloqueado = False
        solidos = [o for o in self.obstaculos if o["tipo"] in ("solo", "edificacao")]
        batidas = [o for o in solidos if circulo_rect(r.x, r.y, r.raio, o["rect"])]
        if batidas:
            for o in batidas:
                self.contatos_frame.add(("robô", "obs", o["id"]))
            r.bloqueado = True
            # tenta deslizar ao longo da superfície (só x ou só y)
            for nx, ny in ((x0 + dx, y0), (x0, y0 + dy), (x0, y0)):
                r.x, r.y = nx, ny
                self.aplicar_barreira(x0, y0)
                if not any(circulo_rect(r.x, r.y, r.raio, o["rect"]) for o in solidos):
                    break
        movido = math.hypot(r.x - x0, r.y - y0)
        r.passo_animacao += movido * 0.25
        if self.viagem_ativa:
            self.metricas.distancia += movido

    def acelerar(self, v_des, dt):
        r = self.robo
        if v_des > r.v:
            r.v = min(v_des, r.v + ACEL_ROBO * dt)
        else:
            r.v = max(v_des, r.v - DESACEL_ROBO * dt)

    def girar_para(self, ang_alvo, dt, taxa=VEL_ANGULAR):
        r = self.robo
        diff = normalizar_angulo(ang_alvo - r.angulo)
        r.angulo += limitar(diff, -taxa * dt, taxa * dt)
        return diff

    # ---------- navegação híbrida ----------
    def chegar(self):
        r = self.robo
        r.v = 0.0
        r.caminho = []
        r.autorizado = False
        r.status_assistivo = f"DESTINO ALCANÇADO: {self.destino_nome}!"
        r.cor_alerta = VERDE_LIVRE
        self.set_estado(DESTINO_ALCANCADO, f"distância ao destino < {RAIO_CHEGADA:.0f} px")
        self.metricas.viagens_concluidas += 1
        self.viagem_ativa = False
        self.t_chegada = self.tempo_sim
        self.emitir("chegada", 0.0, 0.0)

    def navegar(self, dt):
        r, t = self.robo, self.tutor
        self.zona_conflito = None
        if r.estado in (PARADO, DESTINO_ALCANCADO, SEM_ROTA, MANUAL) or not r.caminho:
            self.acelerar(0.0, dt)
            self.mover_robo(r.v * dt)
            return

        fator = self.fator_tensao()
        pts = r.caminho
        trav = r.travessia_atual()
        sem = self.semaforo
        sinal_ok = sem.fase == "PEDESTRE" and sem.tempo_restante > LIMITE_TRAVESSIA
        estado_base = r.estado_antes_freio if r.estado == FREIO_EMERGENCIA else r.estado

        # ===== Camada 0: segurança preditiva (zona varrida) =====
        usar_corredor = r.autorizado or (estado_base == AGUARDANDO_SINAL and sinal_ok)
        if estado_base == AGUARDANDO_SINAL and trav:
            px, py = pts[min(trav["entrada"] + 2, len(pts) - 1)]
            ang_corr = math.atan2(py - r.y, px - r.x)
        else:
            ang_corr = r.angulo
        veic, zona, quem = self.prever_conflito(usar_corredor, ang_corr)
        if veic:
            r.t_ultimo_conflito = self.tempo_sim
            r.veic_conflito = veic
        elif r.estado == FREIO_EMERGENCIA and self.tempo_sim - r.t_ultimo_conflito < 0.4:
            veic = r.veic_conflito   # histerese: mantém o freio 0,4 s após o risco sumir
        if veic:
            self.zona_conflito = zona
            if r.estado != FREIO_EMERGENCIA:
                r.estado_antes_freio = r.estado
                tag = " INFRATOR" if veic.infrator else ""
                self.set_estado(FREIO_EMERGENCIA, f"zona varrida do veículo #{veic.id}{tag} intercepta {quem}")
            r.status_assistivo = "FREIO DE EMERGÊNCIA: veículo em rota de colisão! Parado."
            r.cor_alerta = VERMELHO_PERIGO
            self.acelerar(0.0, dt)
            self.mover_robo(r.v * dt)
            self.emitir("freio", self.pan_relativo(veic.x, veic.y), 0.25)
            return
        if r.estado == FREIO_EMERGENCIA:
            self.set_estado(r.estado_antes_freio, "zona varrida livre: risco eliminado")

        # ===== Aguardando o sinal no meio-fio =====
        if r.estado == AGUARDANDO_SINAL:
            self.acelerar(0.0, dt)
            self.mover_robo(r.v * dt)
            self.girar_para(ang_corr, dt)
            if sinal_ok:
                r.autorizado = True
                self.set_estado(ATRAVESSANDO, f"fase PEDESTRE com {sem.tempo_restante:.1f}s restantes "
                                              f"> {LIMITE_TRAVESSIA:.2f}s e zona varrida livre")
            else:
                if sem.fase == "PEDESTRE":
                    r.status_assistivo = f"Sinal verde acabando ({sem.tempo_restante:.1f}s). Aguardando próximo ciclo."
                else:
                    r.status_assistivo = f"Aguardando sinal de pedestre ({sem.fase}). Parado no meio-fio."
                r.cor_alerta = LARANJA_ALERTA
                return

        # ===== Camada 1: seguimento da rota global (pure pursuit) =====
        limite = len(pts) - 1
        if trav and not r.autorizado:
            limite = trav["espera"]
        self._avancar_indice(limite)

        if trav and r.autorizado and r.idx > trav["saida"] and not em_banda_rua(r.y) and not em_banda_rua(t.y):
            r.autorizado = False
            r.trav_idx += 1
            self.set_estado(NAVEGANDO, "travessia concluída: robô e tutor de volta à calçada")
            trav = r.travessia_atual()
            limite = trav["espera"] if trav else len(pts) - 1

        gx, gy = self.destino_pos
        dist_goal = math.hypot(gx - r.x, gy - r.y)
        if dist_goal < RAIO_CHEGADA and not em_banda_rua(r.y):
            self.chegar()
            return

        if trav and not r.autorizado:
            ex, ey = pts[trav["espera"]]
            if math.hypot(ex - r.x, ey - r.y) < 10:
                self.set_estado(AGUARDANDO_SINAL, f"chegou ao meio-fio da faixa (semáforo: {sem.fase})")
                r.v = 0.0
                return

        if r.estado in (NAVEGANDO, DESVIO_LOCAL):
            px, py = pts[r.idx]
            if math.hypot(px - r.x, py - r.y) > 75:
                self.replanejar("robô se afastou mais de 75 px da rota global")
                return

        alvo = self._ponto_alvo(limite)
        ang_alvo = math.atan2(alvo[1] - r.y, alvo[0] - r.x)
        diff = normalizar_angulo(ang_alvo - r.angulo)
        v_des = VELOCIDADE * fator * max(0.25, math.cos(diff))
        if trav and not r.autorizado:
            ex, ey = pts[trav["espera"]]
            v_des = min(v_des, 2.2 * math.hypot(ex - r.x, ey - r.y) + 12)
        v_des = min(v_des, 2.5 * dist_goal + 20)

        # ===== Camada 2: evasão reativa local (LiDAR) =====
        # (na travessia, o desvio local volta a valer assim que o robô sobe na calçada)
        if r.estado in (NAVEGANDO, DESVIO_LOCAL) or (r.estado == ATRAVESSANDO and not em_banda_rua(r.y)):
            L = r.leituras_lidar
            terco = r.num_raios // 3
            esquerda, centro, direita = L[:terco], L[terco:2 * terco], L[2 * terco:]

            def relevante(l):
                return l["tipo"] not in (None, "veiculo")

            # Histerese: para SAIR do desvio a frente precisa estar livre com folga extra
            histerese = 18.0 if r.estado == DESVIO_LOCAL else 0.0
            perigos = [l for l in centro if relevante(l) and (
                (not l["conhecido"] and l["distancia"] < DIST_DESVIO_DESCONHECIDO + histerese)
                or l["distancia"] < DIST_DESVIO_CONHECIDO + histerese)]
            if r.timer_desvio > 0 and r.idx > r.idx_marco + 5:
                r.timer_desvio = 0.0   # progrediu na rota: o desvio funcionou
            if r.bloqueado:
                r.timer_desvio += dt   # empurrando um obstáculo também conta como "preso"
            if perigos:
                if r.timer_desvio == 0.0:
                    r.idx_marco = r.idx
                mais_perto = min(perigos, key=lambda l: l["distancia"])
                if r.estado != DESVIO_LOCAL:
                    self.set_estado(DESVIO_LOCAL, f"LiDAR: {'obstáculo alto' if mais_perto['tipo'] == 'suspenso' else 'obstáculo'} à frente a "
                                                  f"{mais_perto['distancia']:.0f} px")
                r.timer_desvio += dt
                if r.timer_desvio > TEMPO_MAX_DESVIO:
                    self.replanejar(f"preso em desvio local por mais de {TEMPO_MAX_DESVIO} s")
                    return
                livre_esq = sum(l["distancia"] if relevante(l) else r.alcance_lidar for l in esquerda)
                livre_dir = sum(l["distancia"] if relevante(l) else r.alcance_lidar for l in direita)
                r.angulo += (-1 if livre_esq > livre_dir else 1) * 4.5 * dt
                if mais_perto["tipo"] == "suspenso":
                    r.status_assistivo = "ALERTA CABEÇA: galho/toldo baixo! Desviando..."
                else:
                    r.status_assistivo = "OBSTÁCULO NÃO MAPEADO: contornando..."
                r.cor_alerta = VERMELHO_PERIGO
                # muito perto: gira no lugar; senão avança devagar enquanto contorna
                self.acelerar(0.0 if mais_perto["distancia"] < 36 else VELOCIDADE * 0.3 * fator, dt)
                self.mover_robo(r.v * dt)
                ang_obs = mais_perto["angulo"]
                self.emitir("vermelho", limitar(math.sin(normalizar_angulo(ang_obs - r.angulo)) * 1.4, -1, 1), 0.45)
                return
            if r.estado == DESVIO_LOCAL:
                self.set_estado(NAVEGANDO, "frente livre: retomando a rota global A*")
            if r.timer_desvio > TEMPO_MAX_DESVIO:
                self.replanejar(f"preso (desvio/bloqueio) por mais de {TEMPO_MAX_DESVIO} s")
                return

            min_esq = min((l["distancia"] for l in esquerda if relevante(l)), default=999)
            min_dir = min((l["distancia"] for l in direita if relevante(l)), default=999)
            if min_esq < DIST_LATERAL or min_dir < DIST_LATERAL:
                lado_esq = min_esq < min_dir
                diff += 0.35 if lado_esq else -0.35
                r.status_assistivo = ("Obstáculo à esquerda. Deslocando à direita." if lado_esq
                                      else "Obstáculo à direita. Deslocando à esquerda.")
                r.cor_alerta = LARANJA_ALERTA
                self.emitir("laranja", -1.0 if lado_esq else 1.0, 0.8)
            else:
                r.status_assistivo = (f"Caminho livre. Guiando até {self.destino_nome}"
                                      + (" pelo piso tátil." if self._no_piso_tatil() else "."))
                r.cor_alerta = VERDE_LIVRE
        else:  # ATRAVESSANDO
            r.status_assistivo = "ATRAVESSANDO na faixa de pedestres. Siga em frente."
            r.cor_alerta = CIANO_ROTA
            self.emitir("travessia", 0.0, 0.5)

        r.angulo += limitar(diff, -VEL_ANGULAR * dt, VEL_ANGULAR * dt)
        self.acelerar(v_des, dt)
        self.mover_robo(r.v * dt)

    def _no_piso_tatil(self):
        r = self.robo
        return any(p.inflate(10, 10).collidepoint(r.x, r.y) for p in PISOS_TATEIS)

    def _avancar_indice(self, limite):
        r = self.robo
        melhor, md = r.idx, float("inf")
        for k in range(r.idx, min(limite, r.idx + 12) + 1):
            px, py = r.caminho[k]
            d = math.hypot(px - r.x, py - r.y)
            if d < md:
                melhor, md = k, d
        r.idx = melhor

    def _ponto_alvo(self, limite):
        r = self.robo
        for k in range(r.idx, limite + 1):
            px, py = r.caminho[k]
            if math.hypot(px - r.x, py - r.y) >= LOOKAHEAD:
                return (px, py)
        return r.caminho[limite]

    # ---------- modo manual ----------
    def controle_manual(self, dt, teclas):
        r = self.robo
        fator = self.fator_tensao()
        r.status_assistivo = "Modo manual: setas para conduzir."
        r.cor_alerta = BRANCO
        if teclas is None:
            return
        if teclas[pygame.K_LEFT]:
            r.angulo -= VEL_ANGULAR * 0.8 * dt
        if teclas[pygame.K_RIGHT]:
            r.angulo += VEL_ANGULAR * 0.8 * dt
        alvo_v = VELOCIDADE * fator if teclas[pygame.K_UP] else 0.0
        self.acelerar(alvo_v, dt)
        self.mover_robo(r.v * dt)

    def alternar_modo(self):
        self.modo_autonomo = not self.modo_autonomo
        r = self.robo
        if not self.modo_autonomo:
            r.autorizado = False
            self.set_estado(MANUAL, "ESPAÇO: controle manual pelas setas")
        else:
            r.estado = PARADO
            if self.destino_pos and self.viagem_ativa:
                self.planejar_rota("ESPAÇO: retorno ao modo autônomo")
            else:
                self.historico.append((PARADO, "modo autônomo"))

    # ---------- Marco 4: tutor acoplado pelo arnês ----------
    def atualizar_tutor(self, dt):
        r, t = self.robo, self.tutor
        dx, dy = r.x - t.x, r.y - t.y
        d = math.hypot(dx, dy)
        v_des = min(VEL_TUTOR_MAX, GANHO_TUTOR * (d - GUIA)) if d > GUIA else 0.0
        t.v += limitar(v_des - t.v, -600 * dt, 320 * dt)
        t.v = max(0.0, t.v)
        if d > 1e-6:
            passo = min(t.v * dt, max(0.0, d - 24))
            x0, y0 = t.x, t.y
            mx, my = dx / d * passo, dy / d * passo
            t.x, t.y = x0 + mx, y0 + my
            batidas = [o for o in self.obstaculos if circulo_rect(t.x, t.y, t.raio, o["rect"])]
            if batidas:  # tutor esbarrou: conta contato e desliza pela borda
                for o in batidas:
                    self.contatos_frame.add(("tutor", "obs", o["id"]))
                for nx, ny in ((x0 + mx, y0), (x0, y0 + my), (x0, y0)):
                    t.x, t.y = nx, ny
                    if not any(circulo_rect(t.x, t.y, t.raio, o["rect"]) for o in batidas):
                        break
            d2 = math.hypot(r.x - t.x, r.y - t.y)
            if d2 > GUIA + 30:  # arnês rígido: não estica além disso
                k = (d2 - (GUIA + 30)) / d2
                t.x += (r.x - t.x) * k
                t.y += (r.y - t.y) * k
        r.tensao = math.hypot(r.x - t.x, r.y - t.y) - GUIA

    # ---------- colisões e quase-colisões ----------
    def verificar_contatos(self):
        r, t = self.robo, self.tutor
        atuais = set(self.contatos_frame)
        for o in self.obstaculos:
            if circulo_rect(t.x, t.y, t.raio, o["rect"]):
                atuais.add(("tutor", "obs", o["id"]))
        quase = set()
        for v in self.veiculos:
            rv = v.rect
            for nome, x, y, raio in (("robô", r.x, r.y, r.raio), ("tutor", t.x, t.y, t.raio)):
                gap = dist_ponto_rect(x, y, rv) - raio
                if gap < 0:
                    atuais.add((nome, "veic", v.id))
                elif gap < GAP_QUASE_COLISAO and v.v > 10:
                    quase.add((nome, v.id))
        # Um mesmo contato só conta de novo após 1 s sem encostar (evita contar "raspadas" repetidas)
        agora = self.tempo_sim
        for chave in sorted(atuais, key=str):
            if agora - self.contatos_ativos.get(chave, -99.0) > 1.0:
                self.metricas.colisoes += 1
                if self.trace:
                    print(f"[t={agora:7.2f}s] COLISÃO: {chave}")
            self.contatos_ativos[chave] = agora
        for chave in quase:
            if agora - self.quase_ativos.get(chave, -99.0) > 1.0:
                self.metricas.quase_colisoes += 1
            self.quase_ativos[chave] = agora

    # ---------- passo principal ----------
    def passo(self, dt, teclas=None):
        self.tempo_sim += dt
        self.contatos_frame = set()
        self.semaforo.atualizar(dt)
        self.atualizar_trafego(dt)
        self.robo.ler_lidar(self.obstaculos, self.veiculos, self.rng)
        if self.modo_autonomo:
            self.navegar(dt)
        else:
            self.controle_manual(dt, teclas)
        self.atualizar_tutor(dt)
        self.verificar_contatos()
        if self.viagem_ativa:
            self.metricas.tempo += dt
        if (self.robo.estado == DESTINO_ALCANCADO and self.fila_destinos
                and self.tempo_sim - self.t_chegada > 1.2):
            self.definir_destino(self.fila_destinos.pop(0))


# ==================== RENDERIZAÇÃO ====================
def desenhar_semaforo_veicular(tela, x, y, fase):
    pygame.draw.rect(tela, PRETO, (x, y, 14, 38), border_radius=4)
    vermelho = fase in FASES_VERMELHO_VEICULO
    pygame.draw.circle(tela, (230, 40, 40) if vermelho else (70, 25, 25), (x + 7, y + 7), 5)
    pygame.draw.circle(tela, AMARELO_SINAL if fase == "AMARELO" else (70, 60, 20), (x + 7, y + 19), 5)
    pygame.draw.circle(tela, (40, 220, 90) if fase == "VERDE" else (20, 60, 30), (x + 7, y + 31), 5)


def desenhar_semaforo_pedestre(tela, x, y, sem, fonte):
    pygame.draw.rect(tela, PRETO, (x, y, 18, 30), border_radius=4)
    pedestre = sem.fase == "PEDESTRE"
    pygame.draw.rect(tela, (70, 25, 25) if pedestre else (230, 40, 40), (x + 4, y + 3, 10, 10), border_radius=2)
    pygame.draw.rect(tela, (40, 220, 90) if pedestre else (20, 60, 30), (x + 4, y + 17, 10, 10), border_radius=2)
    if pedestre:
        txt = fonte.render(f"{sem.tempo_restante:.0f}", True, (40, 220, 90))
        tela.blit(txt, (x + 21, y + 8))


def desenhar_ambiente_urbano(tela, sem, fonte_pequena):
    tela.fill(CALCADA_CLARA, (0, 0, AREA_MAPA_LARGURA, ALTURA))
    pygame.draw.rect(tela, GRAMA, (0, 0, AREA_MAPA_LARGURA, 28))
    pygame.draw.rect(tela, GRAMA, (0, ALTURA - 28, AREA_MAPA_LARGURA, 28))
    pygame.draw.rect(tela, ASFALTO, (0, RUA_Y, AREA_MAPA_LARGURA, RUA_ALTURA))
    pygame.draw.line(tela, MEIO_FIO, (0, RUA_Y), (AREA_MAPA_LARGURA, RUA_Y), 4)
    pygame.draw.line(tela, MEIO_FIO, (0, RUA_Y + RUA_ALTURA), (AREA_MAPA_LARGURA, RUA_Y + RUA_ALTURA), 4)
    meio = RUA_Y + RUA_ALTURA // 2
    for x in range(15, AREA_MAPA_LARGURA - 20, 45):
        if not (540 <= x <= 700):
            pygame.draw.line(tela, BRANCO, (x, meio), (x + 22, meio), 3)
    # Faixa de pedestres
    for y in range(RUA_Y + 12, RUA_Y + RUA_ALTURA - 12, 22):
        pygame.draw.rect(tela, BRANCO, (FAIXA_X, y, FAIXA_LARGURA, 12))
    # Linhas de retenção
    pygame.draw.line(tela, BRANCO, (LINHA_RETENCAO_LESTE, meio + 3), (LINHA_RETENCAO_LESTE, RUA_Y + RUA_ALTURA - 3), 4)
    pygame.draw.line(tela, BRANCO, (LINHA_RETENCAO_OESTE, RUA_Y + 3), (LINHA_RETENCAO_OESTE, meio - 3), 4)
    # Piso tátil
    for p in PISOS_TATEIS:
        pygame.draw.rect(tela, PISO_TATIL, p)
    # Semáforos (veiculares no canteiro central, de pedestre nos meio-fios)
    desenhar_semaforo_veicular(tela, 548, meio - 19, sem.fase)
    desenhar_semaforo_veicular(tela, 694, meio - 19, sem.fase)
    desenhar_semaforo_pedestre(tela, 690, RUA_Y - 38, sem, fonte_pequena)
    desenhar_semaforo_pedestre(tela, 690, RUA_Y + RUA_ALTURA + 8, sem, fonte_pequena)


def desenhar_veiculo(tela, v, fonte_pequena, tempo):
    rect = v.rect
    pygame.draw.rect(tela, (25, 25, 30), rect.move(3, 3), border_radius=7)
    pygame.draw.rect(tela, v.cor, rect, border_radius=7)
    # para-brisa na frente
    if v.sentido == 1:
        vidro = pygame.Rect(rect.right - 16, rect.top + 4, 8, rect.height - 8)
        farol_x, lanterna_x = rect.right - 2, rect.left + 2
    else:
        vidro = pygame.Rect(rect.left + 8, rect.top + 4, 8, rect.height - 8)
        farol_x, lanterna_x = rect.left + 2, rect.right - 2
    pygame.draw.rect(tela, (150, 200, 230), vidro, border_radius=2)
    for dy in (4, rect.height - 4):
        pygame.draw.circle(tela, (255, 245, 180), (farol_x, rect.top + dy), 2)
        pygame.draw.circle(tela, (255, 40, 40) if v.freando else (120, 20, 20), (lanterna_x, rect.top + dy), 2)
    if v.infrator:
        if int(tempo * 4) % 2 == 0:
            pygame.draw.rect(tela, AMARELO_SINAL, rect.inflate(4, 4), 2, border_radius=8)
        lbl = fonte_pequena.render("!", True, BRANCO)
        tela.blit(lbl, (rect.centerx - lbl.get_width() // 2, rect.centery - 7))


def quebrar_texto(fonte, texto, largura):
    linhas, atual = [], ""
    for p in texto.split():
        teste = (atual + " " + p).strip()
        if fonte.size(teste)[0] <= largura:
            atual = teste
        else:
            if atual:
                linhas.append(atual)
            atual = p
    if atual:
        linhas.append(atual)
    return linhas


def desenhar_cena(tela, sim, fontes, mostrar_camadas):
    fonte_titulo, fonte_texto, fonte_pequena, fonte_negrito = fontes
    r, t, sem, m = sim.robo, sim.tutor, sim.semaforo, sim.metricas
    desenhar_ambiente_urbano(tela, sem, fonte_pequena)
    overlay = pygame.Surface((AREA_MAPA_LARGURA, ALTURA), pygame.SRCALPHA)

    # C-Space (inflação) e zonas varridas
    if mostrar_camadas:
        cores = {"solo": (255, 255, 255, 50), "suspenso": (155, 89, 182, 70), "edificacao": (255, 120, 0, 50)}
        for rc, tipo in sim.planejador.inflados:
            pygame.draw.rect(overlay, cores[tipo], rc, border_radius=6)
        for v in sim.veiculos:
            if v.v >= 5:
                pygame.draw.rect(overlay, (255, 200, 0, 45), v.zona_varrida())
    if sim.zona_conflito:
        pygame.draw.rect(overlay, (255, 40, 40, 110), sim.zona_conflito)
    tela.blit(overlay, (0, 0))

    # Destinos
    for chave, (nome, pos) in DESTINOS.items():
        ativo = sim.destino_nome == nome and sim.viagem_ativa
        cor = VERDE_LIVRE if ativo else (220, 225, 235)
        pygame.draw.circle(tela, cor, (int(pos[0]), int(pos[1])), 14, 3)
        pygame.draw.circle(tela, cor, (int(pos[0]), int(pos[1])), 4)
        lbl = fonte_pequena.render(nome, True, PRETO)
        fundo = pygame.Rect(0, 0, lbl.get_width() + 8, lbl.get_height() + 2)
        fundo.midbottom = (int(pos[0]), int(pos[1]) - 16)
        fundo.clamp_ip(pygame.Rect(0, 0, AREA_MAPA_LARGURA, ALTURA))
        pygame.draw.rect(tela, cor, fundo, border_radius=4)
        tela.blit(lbl, (fundo.x + 4, fundo.y + 1))
    if sim.destino_nome == "Ponto livre" and sim.destino_pos and sim.viagem_ativa:
        gp = (int(sim.destino_pos[0]), int(sim.destino_pos[1]))
        pygame.draw.circle(tela, VERDE_LIVRE, gp, 18, 3)
        pygame.draw.circle(tela, VERDE_LIVRE, gp, 6)

    # Rota global A*
    if r.caminho and len(r.caminho) - r.idx >= 2:
        pts = [(int(p[0]), int(p[1])) for p in r.caminho[r.idx:]]
        pygame.draw.lines(tela, CIANO_ROTA, False, pts, 2)
        trav = r.travessia_atual()
        if trav and not r.autorizado:
            ex, ey = r.caminho[trav["espera"]]
            pygame.draw.circle(tela, LARANJA_ALERTA, (int(ex), int(ey)), 6, 2)

    # Obstáculos
    for obs in sim.obstaculos:
        if obs["tipo"] == "edificacao":
            continue
        rect = obs["rect"]
        if obs["tipo"] == "suspenso":
            pygame.draw.rect(tela, (90, 95, 102), rect.move(5, 5), border_radius=12)
            pygame.draw.rect(tela, ROXO_SUSPENSO, rect, border_radius=12)
            pygame.draw.rect(tela, BRANCO, rect, 2, border_radius=12)
            texto = "ALTO"
        else:
            pygame.draw.rect(tela, (70, 74, 82), rect, border_radius=5)
            pygame.draw.rect(tela, BRANCO, rect, 2, border_radius=5)
            texto = "OBSTÁCULO"
        if not obs["conhecido"]:
            pygame.draw.rect(tela, LARANJA_ALERTA, rect.inflate(6, 6), 2, border_radius=8)
            texto += "?"
        lbl = fonte_pequena.render(texto, True, BRANCO)
        tela.blit(lbl, (rect.centerx - lbl.get_width() // 2, rect.centery - 7))

    # Veículos
    for v in sim.veiculos:
        desenhar_veiculo(tela, v, fonte_negrito, sim.tempo_sim)

    # Arnês e tutor
    cor_arnes = ARNES_GUIA if r.tensao <= FOLGA_TENSAO else (LARANJA_ALERTA if r.tensao < 20 else VERMELHO_PERIGO)
    pygame.draw.line(tela, cor_arnes, (r.x, r.y), (t.x, t.y), 4)
    pygame.draw.circle(tela, (235, 235, 240), (int(t.x), int(t.y)), t.raio)
    pygame.draw.circle(tela, PRETO, (int(t.x), int(t.y)), t.raio, 2)
    pygame.draw.circle(tela, (60, 60, 70), (int(t.x), int(t.y)), 4)

    r.desenhar(tela)

    # Histórico da máquina de estados (canto superior direito do mapa)
    if sim.historico:
        caixa = pygame.Surface((236, 18 + 15 * len(sim.historico)), pygame.SRCALPHA)
        caixa.fill((15, 17, 22, 185))
        tela.blit(caixa, (AREA_MAPA_LARGURA - 244, 32))
        tela.blit(fonte_pequena.render("Histórico de estados:", True, BRANCO), (AREA_MAPA_LARGURA - 238, 35))
        for i, (est, _) in enumerate(sim.historico):
            tela.blit(fonte_pequena.render(f"{i + 1}. {est}", True, COR_ESTADO.get(est, BRANCO)),
                      (AREA_MAPA_LARGURA - 238, 50 + 15 * i))

    # ===== Painel lateral de telemetria =====
    px = AREA_MAPA_LARGURA
    pygame.draw.rect(tela, PRETO, (px, 0, LARGURA - px, ALTURA))
    pygame.draw.line(tela, BRANCO, (px, 0), (px, ALTURA), 2)
    x = px + 14
    y = 12
    tela.blit(fonte_titulo.render("TELEMETRIA DOG-IA V2", True, CORPO_ROBO), (x, y))
    y += 30

    def linha(txt, cor=BRANCO, fonte=None, passo=18):
        nonlocal y
        tela.blit((fonte or fonte_texto).render(txt, True, cor), (x, y))
        y += passo

    linha(f"Modo: {'AUTÔNOMO (A* + reativo)' if sim.modo_autonomo else 'MANUAL (setas)'}")
    linha(f"Estado: {r.estado}", COR_ESTADO.get(r.estado, BRANCO), fonte_negrito)
    linha(f"Destino: {sim.destino_nome or '-'}")
    cor_fase = {"VERDE": (40, 220, 90), "AMARELO": AMARELO_SINAL, "PEDESTRE": CIANO_ROTA}.get(sem.fase, (230, 60, 60))
    linha(f"Semáforo: {sem.fase} ({sem.tempo_restante:.1f}s)", cor_fase)
    linha(f"Travessia exige > {LIMITE_TRAVESSIA:.2f}s de verde", CINZA_TEXTO)
    min_dist = min((l["distancia"] for l in r.leituras_lidar), default=r.alcance_lidar)
    linha(f"LiDAR filtrado: {min_dist:.0f} px")
    fator = 1.0 if r.tensao <= FOLGA_TENSAO else max(0.15, 1.0 - (r.tensao - FOLGA_TENSAO) / 20.0)
    linha(f"Arnês: tensão {max(0.0, r.tensao):.1f}px  vel {fator * 100:.0f}%", cor_arnes)
    y += 4
    linhas_status = quebrar_texto(fonte_texto, r.status_assistivo, 216)[:3]
    pygame.draw.rect(tela, r.cor_alerta, (x, y, 232, 62), border_radius=8)
    for i, l in enumerate(linhas_status):
        tela.blit(fonte_texto.render(l, True, PRETO), (x + 8, y + 6 + 18 * i))
    y += 74

    linha("MÉTRICAS", CORPO_ROBO, fonte_negrito)
    linha(f"Tempo da viagem: {m.tempo:.1f} s")
    linha(f"Distância: {m.distancia:.0f} px")
    linha(f"Alertas vermelhos: {m.alertas_vermelhos}")
    linha(f"Freios de emergência: {m.freios_emergencia}")
    linha(f"Colisões: {m.colisoes}   Quase: {m.quase_colisoes}")
    linha(f"Replanejamentos A*: {m.replanejamentos}")
    linha(f"Viagens concluídas: {m.viagens_concluidas}")
    y += 8

    linha("CONTROLES", CORPO_ROBO, fonte_negrito)
    for txt in ("1 Mercado   2 Farmácia", "3 Padaria   4 Casa", "5 Tour ida e volta (Mercado/Casa)",
                "Clique esq.: destino livre", "Clique dir.: obstáculo novo",
                "ESPAÇO: Manual / IA   R: reiniciar", "C: C-Space e zonas   M: som"):
        linha(txt, BRANCO, fonte_pequena, 16)
    y += 8

    linha("LEGENDA", CORPO_ROBO, fonte_negrito)
    itens = [(PISO_TATIL, "Piso tátil (custo 0.45)"), ((70, 74, 82), "Obstáculo"),
             (ROXO_SUSPENSO, "Galho/toldo (alto)"), (VERMELHO_INFRATOR, "Veículo infrator"),
             (CIANO_ROTA, "Rota global A*")]
    for cor, txt in itens:
        pygame.draw.rect(tela, cor, (x, y + 3, 12, 10), border_radius=2)
        tela.blit(fonte_pequena.render(txt, True, BRANCO), (x + 18, y))
        y += 16


# ==================== EXECUÇÃO INTERATIVA ====================
def main_interativo(seed):
    pygame.mixer.pre_init(22050, -16, 2, 512)
    pygame.init()
    tela = pygame.display.set_mode((LARGURA, ALTURA))
    pygame.display.set_caption("DOG-IA V2 — Simulador Urbano de Cão-Guia Autônomo")
    clock = pygame.time.Clock()
    fontes = (pygame.font.SysFont("arial", 19, bold=True), pygame.font.SysFont("arial", 14),
              pygame.font.SysFont("arial", 12, bold=True), pygame.font.SysFont("arial", 14, bold=True))
    audio = AudioEstereo()
    sim = Simulacao(seed=seed, imprimir_estados=True)
    mostrar_camadas = False

    rodando = True
    while rodando:
        dt = min(clock.tick(FPS) / 1000.0, 1.0 / 20.0)   # dt em segundos, independente do FPS
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                rodando = False
            elif evento.type == pygame.MOUSEBUTTONDOWN:
                mx, my = evento.pos
                if mx < AREA_MAPA_LARGURA:
                    if evento.button == 1:
                        sim.fila_destinos = []
                        sim.definir_destino((mx, my))
                    elif evento.button == 3:
                        sim.adicionar_obstaculo(mx, my, "solo")
            elif evento.type == pygame.KEYDOWN:
                if evento.key == pygame.K_ESCAPE:
                    rodando = False
                elif evento.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4,
                                    pygame.K_KP1, pygame.K_KP2, pygame.K_KP3, pygame.K_KP4):
                    n = {pygame.K_1: 1, pygame.K_2: 2, pygame.K_3: 3, pygame.K_4: 4,
                         pygame.K_KP1: 1, pygame.K_KP2: 2, pygame.K_KP3: 3, pygame.K_KP4: 4}[evento.key]
                    sim.fila_destinos = []
                    sim.definir_destino(n)
                elif evento.key in (pygame.K_5, pygame.K_KP5):
                    sim.fila_destinos = [4]
                    sim.definir_destino(1)
                elif evento.key == pygame.K_SPACE:
                    sim.alternar_modo()
                elif evento.key == pygame.K_r:
                    sim = Simulacao(seed=seed, imprimir_estados=True)
                elif evento.key == pygame.K_c:
                    mostrar_camadas = not mostrar_camadas
                elif evento.key == pygame.K_m:
                    audio.mudo = not audio.mudo

        sim.passo(dt, pygame.key.get_pressed())
        for tipo, pan in sim.eventos_som:
            audio.tocar(tipo, pan)
        sim.eventos_som.clear()

        desenhar_cena(tela, sim, fontes, mostrar_camadas)
        pygame.display.flip()

    pygame.quit()


# ==================== EXECUÇÃO HEADLESS ====================
def sortear_obstaculo(sim, destino_pos):
    """Obstáculo desconhecido aleatório na calçada (não mapeado pelo planejador)."""
    r = sim.robo
    for _ in range(60):
        if sim.rng.random() < 0.5:
            x, y = sim.rng.uniform(40, AREA_MAPA_LARGURA - 40), sim.rng.uniform(60, RUA_Y - 40)
        else:
            x, y = sim.rng.uniform(40, AREA_MAPA_LARGURA - 40), sim.rng.uniform(RUA_Y + RUA_ALTURA + 40, ALTURA - 60)
        if FAIXA_X - 60 < x < FAIXA_X + FAIXA_LARGURA + 60 and (abs(y - RUA_Y) < 90 or abs(y - RUA_Y - RUA_ALTURA) < 90):
            continue  # não bloqueia a cabeceira da faixa
        longe = all(math.hypot(x - p[0], y - p[1]) > 80 for _, p in DESTINOS.values())
        longe = longe and math.hypot(x - destino_pos[0], y - destino_pos[1]) > 80
        longe = longe and math.hypot(x - r.x, y - r.y) > 90 and math.hypot(x - sim.tutor.x, y - sim.tutor.y) > 60
        rect = pygame.Rect(int(x - 20), int(y - 20), 40, 40)
        livre = not any(rect.inflate(20, 20).colliderect(o["rect"]) for o in sim.obstaculos if o["tipo"] != "edificacao")
        if longe and livre:
            return sim.adicionar_obstaculo(x, y, "solo", 40)
    return None


def executar_viagem(sim, chave, timeout=120.0):
    inicio = sim.metricas.copia()
    sim.definir_destino(chave)
    t0 = sim.tempo_sim
    sucesso = False
    while True:
        sim.passo(DT_BENCHMARK)
        if sim.robo.estado == DESTINO_ALCANCADO:
            sucesso = True
            break
        if sim.robo.estado == SEM_ROTA or sim.tempo_sim - t0 > timeout:
            break
    fim = sim.metricas.copia()
    delta = {c: fim[c] - inicio[c] for c in Metricas.CAMPOS}
    delta["tempo"] = sim.tempo_sim - t0
    delta["distancia"] = sim.metricas.distancia
    return sucesso, delta


def executar_benchmark(n, seed, trace):
    sim = Simulacao(seed=seed, trace=trace)
    print(f"=== DOG-IA V2 — BENCHMARK headless: {n} viagens, seed={seed} ===")
    print(f"{'#':>3}  {'origem':<9} -> {'destino':<9} {'result.':<8} {'tempo':>7} {'dist':>6} "
          f"{'col':>4} {'quase':>5} {'freio':>5} {'alert':>5} {'repl':>4} {'obs+':>4}")
    atual = 4
    resultados = []
    for i in range(1, n + 1):
        opcoes = [k for k in DESTINOS if k != atual]
        destino = sim.rng.choice(opcoes)
        extras = []
        for _ in range(sim.rng.choice([0, 1, 1, 2])):
            o = sortear_obstaculo(sim, DESTINOS[destino][1])
            if o:
                extras.append(o)
        origem = DESTINOS[atual][0] if atual else "(meio)"
        if trace:
            print(f"--- viagem {i}: {origem} -> {DESTINOS[destino][0]} ---")
        sucesso, d = executar_viagem(sim, destino)
        resultados.append((sucesso, d))
        print(f"{i:>3}  {origem:<9} -> {DESTINOS[destino][0]:<9} {'OK' if sucesso else 'FALHA':<8} "
              f"{d['tempo']:>6.1f}s {d['distancia']:>6.0f} {d['colisoes']:>4} {d['quase_colisoes']:>5} "
              f"{d['freios_emergencia']:>5} {d['alertas_vermelhos']:>5} {d['replanejamentos']:>4} {len(extras):>4}")
        for o in extras:
            sim.obstaculos.remove(o)
        atual = destino if sucesso else None
        if not sucesso:
            # recomeça do ponto onde parou; reseta estado
            sim.robo.estado = PARADO
            sim.robo.caminho = []
            sim.viagem_ativa = False

    ok = [d for s, d in resultados if s]
    tot = lambda campo: sum(d[campo] for _, d in resultados)  # noqa: E731
    print("-" * 86)
    print(f"Taxa de sucesso.......: {len(ok)}/{n} ({100.0 * len(ok) / n:.1f}%)")
    if ok:
        print(f"Tempo médio (sucesso).: {sum(d['tempo'] for d in ok) / len(ok):.2f} s")
        print(f"Distância média.......: {sum(d['distancia'] for d in ok) / len(ok):.0f} px")
    print(f"Colisões..............: {tot('colisoes')}")
    print(f"Quase-colisões........: {tot('quase_colisoes')}")
    print(f"Freios de emergência..: {tot('freios_emergencia')}")
    print(f"Alertas vermelhos.....: {tot('alertas_vermelhos')}")
    print(f"Replanejamentos A*....: {tot('replanejamentos')}")
    print(f"Tempo simulado total..: {sim.tempo_sim:.1f} s")


def executar_viagem_unica(chave, seed):
    sim = Simulacao(seed=seed, trace=True)
    nome = DESTINOS[chave][0]
    print(f"=== Viagem headless Casa -> {nome} (seed={seed}) ===")
    sucesso, d = executar_viagem(sim, chave)
    print(f"Resultado: {'OK' if sucesso else 'FALHA'} | tempo {d['tempo']:.1f}s | distância {d['distancia']:.0f}px | "
          f"freios {d['freios_emergencia']} | colisões {d['colisoes']} | quase {d['quase_colisoes']}")


def main():
    p = argparse.ArgumentParser(description="DOG-IA V2 — Simulador Urbano de Cão-Guia Autônomo")
    p.add_argument("--benchmark", type=int, nargs="?", const=20, default=None,
                   help="executa N viagens sem interface gráfica (padrão 20)")
    p.add_argument("--seed", type=int, default=None, help="semente do gerador aleatório")
    p.add_argument("--trace", action="store_true", help="imprime as transições de estado no benchmark")
    p.add_argument("--viagem", type=int, choices=[1, 2, 3, 4], default=None,
                   help="executa uma única viagem headless a partir da Casa, com log de estados")
    args = p.parse_args()
    if args.benchmark:
        executar_benchmark(args.benchmark, args.seed, args.trace)
    elif args.viagem:
        executar_viagem_unica(args.viagem, args.seed)
    else:
        main_interativo(args.seed)


if __name__ == "__main__":
    main()
