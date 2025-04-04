import paramiko

class FileUploader:
    host = "HOST_NAME"
    port = 22
    username = "pi"
    password = "yahboom"
    remote_path = "PATH_TO_FILE"
    local_path = "PATH_TO_LOCAL"

    def __init__(self, remote, local):
        self.remote_path = remote
        self.local_path = local

    def upload(self):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(self.host, self.port, self.username, self.password)
        sftp = client.open_sftp()
        sftp.put(self.local_path, self.remote_path)
        sftp.close()
        client.close()