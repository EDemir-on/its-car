document.addEventListener('DOMContentLoaded', function() {
    document.addEventListener('keydown', function(event) {
        let key = event.key;

        // Send HTTP requests based on arrow key presses
        if (key === 'ArrowUp') {
            fetch('/move/forward', { method: 'POST' });
        } else if (key === 'ArrowDown') {
            fetch('/move/backward', { method: 'POST' });
        } else if (key === 'ArrowLeft') {
            fetch('/turn/left', { method: 'POST' });
        } else if (key === 'ArrowRight') {
            fetch('/turn/right', { method: 'POST' });
        } else if (key === ' ') {
            fetch('/stop', { method: 'POST' });
        }
    });
});