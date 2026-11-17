# Projeto DOG-IA — AC-4: respostas às 5 questões técnicas

> Todos os números abaixo saíram do próprio `dog_ia_v2.py` (saídas salvas em
> `bench_seed1.txt`, `bench_seed7.txt` e `viagem_mercado.txt`).

---

## 1. Arquitetura híbrida de navegação (global vs. local)

**Qual limitação da navegação puramente reativa a arquitetura híbrida resolve?**
A V1 decide tudo olhando só a leitura atual do LiDAR e o ângulo até o alvo. Por isso ela
fica presa em **mínimos locais**: diante de um obstáculo em "U", de um beco ou de uma
fileira de obstáculos, a regra "desvia para o lado mais livre e volta a mirar no alvo"
faz o robô oscilar ou parar. Ela também não tem noção do mapa. Não sabe que deve
atravessar **só na faixa**, não prefere o **piso tátil** e não escolhe a melhor rota entre
vários destinos. Na V2, o **A\*** sobre a grade de ocupação (com C-Space e custo 0,45 no
piso tátil) calcula de antemão uma rota completa, sem colisão e acessível. A camada
reativa só cuida do que o mapa não conhece.

**Por que o planejador global sozinho não basta em um ambiente urbano?**
O A\* trabalha sobre um **mapa estático e conhecido a priori**. A rua tem coisas
**dinâmicas e imprevistas**: veículos em movimento, motoristas infratores, obstáculos novos
(lixeira fora do lugar, pessoas), ruído do sensor e erro de posição. Replanejar o A\* a
60 Hz para cada mudança custa caro, e a rota ainda ficaria desatualizada entre um
replanejamento e outro. Por isso a V2 usa camadas:
**segurança preditiva (zona varrida) → regras de travessia (semáforo/meio-fio) → evasão
reativa local (LiDAR) → rota global (A\*)**. Se o robô fica preso no desvio local por
mais de 3,5 s, ele replaneja e inclui no mapa os obstáculos que percebeu.

---

## 2. Máquina de estados

Viagem **1 (Mercado)** saindo de Casa, com `python dog_ia_v2.py --viagem 1 --seed 1`
(as mesmas transições aparecem no painel "Histórico de estados" da simulação gráfica):

```
[t=   0.00s] PARADO -> PLANEJANDO | destino 'Mercado' selecionado
[t=   0.00s] PLANEJANDO -> NAVEGANDO | rota A* com 115 nós (custo 1195), 1 travessia(s)
[t=   4.05s] NAVEGANDO -> AGUARDANDO_SINAL | chegou ao meio-fio da faixa (semáforo: PEDESTRE)
[t=   4.07s] AGUARDANDO_SINAL -> ATRAVESSANDO | fase PEDESTRE com 4.3s restantes > 2.44s e zona varrida livre
[t=   5.93s] ATRAVESSANDO -> NAVEGANDO | travessia concluída: robô e tutor de volta à calçada
[t=   8.70s] NAVEGANDO -> DESTINO_ALCANCADO | distância ao destino < 18 px
```

### 2.1 Estados, em ordem, e a condição de cada transição

| # | Transição | Condição que causou |
|---|-----------|---------------------|
| 1 | `PARADO → PLANEJANDO` | Tecla **1** escolheu o destino Mercado. |
| 2 | `PLANEJANDO → NAVEGANDO` | O A\* achou uma rota no C-Space (115 nós, 1 travessia detectada na rota). |
| 3 | `NAVEGANDO → AGUARDANDO_SINAL` | O robô chegou ao **ponto de espera no meio-fio** (último nó antes da pista). A barreira não deixa descer sem autorização. |
| 4 | `AGUARDANDO_SINAL → ATRAVESSANDO` | Fase `PEDESTRE` **e** `tempo_restante > 2,44 s` **e** nenhuma zona varrida de veículo cruzando o corredor de travessia. |
| 5 | `ATRAVESSANDO → NAVEGANDO` | O robô passou o último nó da pista **e** robô e tutor saíram da banda da avenida. |
| 6 | `NAVEGANDO → DESTINO_ALCANCADO` | Distância ao destino < 18 px. |

Observações:
* Com a seed 1 o robô chegou ao meio-fio com o sinal de pedestre já aberto e 4,3 s
  restantes, então esperou só um quadro. Com a seed 7 (`--viagem 1 --seed 7`) ele chega
  no fim da fase de pedestre, com menos de 2,44 s restantes. Por isso não atravessa e fica
  **15 s em AGUARDANDO_SINAL** (de 4,05 s até 19,07 s), esperando o próximo ciclo.
* Se houver um obstáculo novo na calçada, aparece `NAVEGANDO ⇄ DESVIO_LOCAL`, e um
  `PLANEJANDO` extra depois de 3,5 s preso. Se aparecer um infrator,
  surge `FREIO_EMERGENCIA`.

### 2.2 Diferença entre AGUARDANDO_SINAL e FREIO_EMERGENCIA

| | **AGUARDANDO_SINAL** | **FREIO_EMERGENCIA** |
|---|---|---|
| Natureza | **Normativa e planejada**: é uma etapa prevista da rota | **Reativa e preditiva**: é uma exceção de segurança |
| Gatilho | Chegar ao meio-fio da faixa sem autorização (sinal fechado ou verde acabando) | A **zona varrida** de um veículo (1,0 s à frente) intercepta o robô, o tutor ou o corredor de travessia |
| Depende do semáforo? | Sim, sai quando abre `PEDESTRE` com tempo suficiente | **Não**, vale mesmo com o sinal verde para pedestres (é o caso do **infrator**) |
| Onde ocorre | Só no ponto de espera, na calçada | Em qualquer ponto: no meio-fio ou **no meio da travessia** |
| Saída | Vai para `ATRAVESSANDO` | Volta ao **estado anterior** quando a zona fica livre (com 0,4 s de histerese) |
| Prioridade | Normal | Máxima: é avaliada antes de todas as outras camadas |

Resumo: em AGUARDANDO_SINAL o robô **obedece à regra de trânsito**. Em FREIO_EMERGENCIA ele
**se protege de quem desrespeita a regra**.

---

## 3. Percepção e ruído do sensor

### 3.1 Por que a mediana é melhor que a média para falhas "leitura = alcance máximo"?
O dropout gera um **outlier extremo** (165 px) no meio de leituras corretas. A média é
puxada por ele. Com um obstáculo real a 30 px, a janela `[30, 30, 165]` dá
**média = 75 px**: o robô "acha" que o obstáculo está 2,5 vezes mais longe e pode não
desviar ou não frear. A **mediana** da mesma janela é **30 px**, porque o valor extremo
fica numa ponta da ordenação e é descartado. A mediana tem *breakdown point* de 50 %:
ela aguenta até ⌊N/2⌋ leituras erradas na janela (1 de 3 aqui). A média tem
*breakdown* de 0 %, então uma única leitura ruim já corrompe o resultado. Além disso, a
mediana devolve sempre um valor que **foi medido de verdade**. Ela não inventa uma
distância intermediária que não existe.

### 3.2 Custo do filtro em atraso (60 varreduras/s)
Quando um obstáculo aparece de repente, a saída da mediana de 3 só "vira" quando **2 das 3**
leituras o enxergam. Isso dá **1 varredura de atraso**:

* atraso típico = (N − 1)/2 = 1 varredura = 1/60 s ≈ **0,017 s (≈ 17 ms)**;
* pior caso, com um dropout logo em seguida: 2 varreduras ≈ **0,033 s**;
* a 156 px/s isso equivale a ≈ 2,6 px (até 5,2 px) de deslocamento, bem abaixo das
  margens do C-Space (22 a 38 px).

### 3.3 E se a janela fosse de 15 varreduras?
* **Atraso**: (15 − 1)/2 = 7 varreduras ≈ **0,117 s**. Nesse tempo o robô anda ≈ 18 px e um
  carro a 200 px/s anda ≈ 23 px. A margem de segurança efetiva diminui.
* **Objetos rápidos ou finos somem**: algo visto em menos de 8 das 15 varreduras (um carro
  cruzando o feixe, uma pessoa passando) **não aparece** na saída.
* **Borrão na rotação**: os raios estão no referencial do robô. Quando ele gira, a janela
  mistura direções diferentes, e o filtro passa a responder sobre um lugar onde o robô
  já não está olhando.
* Vantagem: aguentaria rajadas de até 7 falhas seguidas, mas com 2 % de dropout isso
  quase nunca acontece. Para esse sensor o custo não compensa. O certo seria aumentar
  as margens ou reduzir a velocidade se a janela fosse mesmo de 15.

---

## 4. Segurança temporal na travessia

### 4.1 Cálculo do limite
```
limite = (RUA_ALTURA + 24 + GUIA) / velocidade × 1,35 + 0,4
       = (170 + 24 + 42) / 156 × 1,35 + 0,4
       = 236 / 156 × 1,35 + 0,4
       = 1,5128 × 1,35 + 0,4
       = 2,0423 + 0,4
       ≈ 2,44 s
```
O robô só começa a travessia se faltarem **mais de 2,44 s** de sinal verde para pedestres.
O numerador cobre a pista inteira, a folga e o comprimento do arnês, porque o **tutor**
também precisa sair da pista. O fator 1,35 cobre a desaceleração causada pela tensão do
arnês e as curvas. Os 0,4 s cobrem o tempo de reação e a latência do sensor e do filtro.

### 4.2 Por que existem fases "todos vermelhos" de 1,5 s?
São **intervalos de limpeza** (*clearance*). Nesse tempo nenhum fluxo tem verde:

* **TODOS_VERM_1 (depois do amarelo)**: um carro que estava na **zona de dilema** do
  amarelo (perto demais para parar) ainda está passando pela faixa. A 160 px/s, cruzar a
  faixa mais o próprio comprimento (≈ 95 + 46 + 28 px) leva ≈ 1,06 s, que cabe em 1,5 s.
  Sem essa fase, o verde do pedestre abriria com um carro ainda em cima da faixa.
* **TODOS_VERM_2 (depois do verde de pedestre)**: quem começou a atravessar no fim do
  verde (ou anda mais devagar, como o tutor) termina a travessia antes de os carros
  receberem verde.

O risco eliminado é o **conflito entre o último veículo e o primeiro pedestre (e vice-versa)**:
duas permissões que se sobrepõem por tempo de reação, frenagem ou atraso de quem está
cruzando. Na simulação isso aparece assim: carros que "passaram no amarelo" (`passa_amarelo`)
saem da faixa durante o TODOS_VERM_1.

---

## 5. Avaliação experimental

Comandos: `python dog_ia_v2.py --benchmark 20 --seed 1` e `... --seed 7`.
Cada viagem sorteia um destino diferente do atual e coloca de 0 a 2 **obstáculos
desconhecidos** na calçada. O planejador não conhece esses obstáculos, só o LiDAR os vê.

### 5.1 Resultados

| Métrica | seed 1 | seed 7 |
|---|---|---|
| Taxa de sucesso | **20/20 (100 %)** | **20/20 (100 %)** |
| Tempo médio por viagem | **11,07 s** | **16,14 s** |
| Distância média | 770 px | 890 px |
| Colisões | **6** | **5** |
| Quase-colisões (folga < 12 px de veículo em movimento) | **1** | **0** |
| Freios de emergência | 1 | 2 |
| Alertas vermelhos (entradas em DESVIO_LOCAL) | 69 | 226 |
| Replanejamentos A\* | 2 | 12 |

As colisões foram todas **encostadas do robô ou do tutor em obstáculos desconhecidos**
colocados pelo benchmark, durante o desvio local. **Nenhuma colisão com veículo**
aconteceu. A única quase-colisão (viagem 6 da seed 1) foi um **infrator** que avançou o
vermelho durante a travessia: a zona varrida dele atingiu o tutor, o robô entrou em
`FREIO_EMERGENCIA` e o carro passou sem encostar.

### 5.2 Reprodutibilidade e por que as seeds diferem
* O simulador usa **um único gerador pseudoaleatório** (`random.Random(seed)`) para tudo:
  fase inicial do semáforo, chegada de carros, quem é infrator, velocidade máxima, ruído
  gaussiano e dropout do LiDAR, destinos e obstáculos sorteados. Um PRNG é
  **determinístico**: a mesma semente produz sempre a mesma sequência. Além disso, o
  benchmark usa **dt fixo = 1/60 s**, que não depende do relógio nem do FPS do computador.
  Resultado: a mesma seed gera exatamente a mesma saída (rodamos duas vezes a seed 1 e o
  `diff` deu vazio).
* Seeds diferentes geram **sequências diferentes**, ou seja, **cenários diferentes**:
  outros destinos, outra fase do semáforo quando o robô chega ao meio-fio (a espera pode
  ir de 0 a ≈ 15 s e domina o tempo médio), outros obstáculos (alguns caem bem no caminho
  e geram oscilação no desvio, como os 142 alertas e 10 replanejamentos da viagem 4 da
  seed 7) e outro tráfego, com infratores aparecendo em momentos diferentes. Com 20 viagens a variância
  amostral é alta. Para comparar versões do código o certo é rodar **várias seeds** e
  reportar média ± desvio.

### 5.3 Melhoria proposta
**Inserir na grade de ocupação, na hora, todo obstáculo "não mapeado" que o LiDAR detectar,
e replanejar o A\* logo em seguida (replanejamento incremental, estilo D\* Lite), em vez de
esperar 3,5 s preso no desvio local.**

* **Deve melhorar**: **colisões** (as 11 registradas foram justamente contra obstáculos
  desconhecidos durante o desvio reativo), **alertas vermelhos** (some a oscilação
  NAVEGANDO ⇄ DESVIO_LOCAL: 69 na seed 1 e 226 na seed 7) e o tempo das viagens afetadas.
* **Pode piorar**: o número de **replanejamentos** e o custo de CPU por quadro. A rota pode
  ficar um pouco **mais longa** (maior distância média), porque o obstáculo inflado em
  30–38 px fecha passagens estreitas. E se o LiDAR tiver um falso positivo que passe pela
  mediana, o robô desvia de um obstáculo que não existe.

Alternativa mais simples: aumentar `DIST_DESVIO_DESCONHECIDO` de 58 para 80 px. Isso reduz
colisões, mas aumenta os alertas e o tempo médio, porque o robô passa a reagir mais cedo e
com mais frequência.
