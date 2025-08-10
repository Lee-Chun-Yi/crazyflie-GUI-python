def main():
    # 延遲匯入，啟動更快也避免不必要副作用
    from .ui import App
    App().mainloop()

if __name__ == "__main__":
    main()