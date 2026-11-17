# DOG-IA: cão-guia robótico autônomo (AC-4)

Simulador em Python/pygame de um cão-guia robótico que conduz uma pessoa com deficiência
visual pela cidade. Ele respeita a faixa de pedestres e o semáforo, desvia de obstáculos de
solo e de obstáculos **na altura da cabeça** do tutor (galhos e toldos) e freia diante de
motoristas que avançam o sinal.

| Arquivo | Conteúdo |
|---|---|
| `dog_ia_v1.py` | Código base (navegação só reativa) |
| `dog_ia_v2.py` | Versão final com os 4 marcos |
| `RESPOSTAS.md` | Respostas às 5 questões técnicas |
| `bench_seed1.txt`, `bench_seed7.txt`, `viagem_mercado.txt` | Saídas usadas nas respostas |

## Como rodar (VS Code)
1. Abra a pasta no VS Code (*File → Open Folder*).
2. No terminal (*Terminal → New Terminal*):
   ```
   pip install pygame-ce
   python dog_ia_v2.py
   ```
   (ou abra `dog_ia_v2.py` e clique em ▶ *Run Python File*)

### Controles
| Tecla | Ação |
|---|---|
| 1 / 2 / 3 / 4 | Destino: Mercado / Farmácia / Padaria / Casa |
| 5 | Tour ida e volta: Casa → Mercado → Casa (para o vídeo bônus) |
| Clique esquerdo | Destino livre |
| Clique direito | Cria obstáculo **desconhecido** (testa desvio local + replanejamento) |
| ESPAÇO | Alterna entre manual (setas) e IA |
| C | Mostra o C-Space (inflação) e as zonas varridas dos veículos |
| M | Liga/desliga o som estéreo |
| R | Reinicia |

### Modo headless
```
python dog_ia_v2.py --benchmark 20 --seed 1
python dog_ia_v2.py --benchmark 20 --seed 7
python dog_ia_v2.py --viagem 1 --seed 1        # 1 viagem Casa -> Mercado com log da máquina de estados
python dog_ia_v2.py --benchmark 5 --seed 1 --trace
```

## O que mudou da V1 para a V2
* **Marco 1**: semáforo `VERDE → AMARELO → TODOS_VERM_1 → PEDESTRE → TODOS_VERM_2` em tempo
  simulado (`dt`). Carros nas duas mãos com `ACEL`/`DESACEL`, distância de seguimento e parada
  na linha de retenção. Veículos **infratores** (vermelhos, piscando) e `aplicar_barreira()`
  no meio-fio.
* **Marco 2**: grade 10×10 px + **A\*** (heurística octile), C-Space assimétrico (solo 30 px,
  edificação 22 px, suspenso 38 px), custo **0,45** no piso tátil, destinos 1–4 e
  replanejamento depois de 3,5 s preso.
* **Marco 3**: LiDAR com `rect.clipline`, ruído gaussiano σ = 1,5 px, dropout de 2 %, mediana
  temporal `deque(maxlen=3)` e predição por **zona varrida** (Δt = 1 s) que aciona `FREIO_EMERGENCIA`.
* **Marco 4**: tutor como entidade própria no arnês (`GUIA = 42 px`), com redução de velocidade
  quando a tensão passa de 10 px. Bipes estéreo com pan de −1 a +1, métricas e `--benchmark`.

## Rascunho de post para o LinkedIn
> 🦮🤖 **DOG-IA: um cão-guia robótico para uma cidade mais acessível**
>
> No Brasil, milhões de pessoas têm deficiência visual, e um cão-guia treinado custa caro e
> leva anos para ficar pronto. Nosso grupo desenvolveu o **DOG-IA**, um simulador de cão-guia
> autônomo que planeja rotas pelo piso tátil (A\*), espera o sinal de pedestre, protege a
> cabeça do tutor de galhos e toldos e **freia sozinho quando um carro avança o vermelho**.
>
> O objetivo é mostrar que IA e robótica podem devolver **autonomia e segurança** a quem
> mais precisa delas no dia a dia.
>
> Projeto da disciplina com o prof. Flavio Santarelli. Equipe: @… @… @…
> #InteligenciaArtificial #Acessibilidade #Robotica #Python #Inclusao
