# LapTimeSimulator_CopaTruck

A engenharia automotiva moderna exige simulações eficientes para analisar o desempenho dinâmico. Em competições de caminhões como a Copa Truck, otimizar o tempo de volta é vital para validar técnicas, refinar estratégias e inovar.

## Estrutura do Projeto

```
LapTimeSimulator_CopaTruck/
├── docs/           # Documentação do projeto
├── src/            # Código fonte
│   ├── __init__.py
│   ├── modelo_dinamico.py    # Modelo dinâmico do veículo
│   ├── controle.py           # Sistema de controle
│   ├── simulacao.py          # Motor de simulação
│   ├── interface.py          # Interface Streamlit
│   ├── exportacao.py         # Exportação de resultados
│   └── utils.py              # Funções utilitárias
├── data/           # Dados de entrada e configurações
├── results/        # Resultados das simulações
└── requirements.txt
```

## Instalação

### Pré-requisitos
- Python 3.8 ou superior
- pip (gerenciador de pacotes Python)

### Instruções de Setup

1. Clone o repositório:
```bash
git clone https://github.com/vitormtt/LapTimeSimulator_CopaTruck.git
cd LapTimeSimulator_CopaTruck
```

2. Instale as dependências:
```bash
pip install -r requirements.txt
```

## Como Usar

### Executar a Interface Streamlit

Para iniciar o simulador com interface gráfica:

```bash
streamlit run src/interface.py
```

A aplicação abrirá automaticamente no seu navegador padrão (normalmente em `http://localhost:8501`).

## Funcionalidades

- **Modelo Dinâmico**: Simulação de forças, aceleração e dinâmica do veículo
- **Sistema de Controle**: Otimização de aceleração e frenagem
- **Simulação de Volta**: Cálculo completo de tempo de volta
- **Interface Interativa**: Visualização e controle via Streamlit
- **Exportação de Dados**: Resultados em CSV, JSON e Excel
- **Utilitários**: Conversões e funções auxiliares

## Comandos Git

Comandos básicos para trabalhar com o repositório:

```bash
# Verificar status
git status

# Adicionar arquivos
git add .

# Fazer commit
git commit -m "Descrição das alterações"

# Enviar para o repositório remoto
git push

# Atualizar do repositório remoto
git pull
```

## Contribuindo

Contribuições são bem-vindas! Sinta-se à vontade para abrir issues e pull requests.

## Licença

Este projeto está em desenvolvimento para fins acadêmicos e de pesquisa.
