from wobl.sim.balance_task import BalanceTask
from wobl.sim.application import Application


def main():
    task = BalanceTask()
    app = Application(task)
    app.launch()


if __name__ == "__main__":
    main()
