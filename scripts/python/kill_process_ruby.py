import psutil


def main():
    proc_list = list(filter(lambda x: x.name() == "ruby", psutil.process_iter()))
    for p in proc_list:
        print(f'Found <ruby> process (pid: {p.pid})...')
        p.kill()


if __name__ == '__main__':
    main()
