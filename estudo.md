# Estudo: Funcionamento do Yaw (IMU) - LAB 6

O **yaw** representa a rotação em torno do eixo **Z**. Sem um **magnetômetro** (bússula), ele não tem referência do norte — o yaw **drifta com o tempo**.

**Fonte do Drift:** Ele é calculado em uma integral no tempo dos dados lidos no giroscópio. Essa integral gera erros acumulativos que com o tempo podem ser perceptíveis(No mouse, podemos ir mexendo manualmente ele de forma que o humano compense esse erro)

### Correção automática (que ocorre no roll e pitch):
- **Roll** e **pitch** podem ser corrigidos com a gravidade (acelerômetro).
- **Para o Yaw é mais complicado** — está perpendicular a gravidade, ou seja, a movimentação em seu eixo não é afetada pela aceleração perpendicular ao sentido do movimento.

Ou seja, por algum motivo o meu trabalho filtro ahrs não quer fazer a conta do jeito certo pro yaw e gera um erro. O Yaw (do euler) fica sempre = 0.

**Solução:** A correção é forçar a integral. Então eu integro de forma numérica (por uma Soma de Riemann - retângulos de dimensões dt e gyro.axis.z) de forma a aproximar o valor que o yaw deveria ter (que deveria ser analisado com a aceleração). O problema é que o erro acumulativo piora muito em pouco tempo. Mas, funciona.