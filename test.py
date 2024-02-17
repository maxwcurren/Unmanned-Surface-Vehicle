def receive_Lora(max_retries=5):
    retry_count = 0

    while retry_count < max_retries:
        ser.write(('+RCV\r\n').encode('utf-8'))
        response = ser.readline().decode('utf-8').strip()

        if response:
            if 'ERR' in response:
                print(f"Received error: {response}")
            else:
                print(f"Received: {response}")
                return response
        retry_count += 1
        print(f"Retrying... {retry_count}/{max_retries}")
        time.sleep(0.1)

    print(f"Exceeded maximum retries ({max_retries})")
